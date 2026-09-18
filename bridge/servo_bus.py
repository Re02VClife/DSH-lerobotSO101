"""SO-101 / Feetech STS3215 bus access with hard safety rails.

Design rules (do not relax without a hardware e-stop in the loop):
  * default mode is READ-ONLY; motion requires an explicit constructor flag;
  * every motion is clamped to per-joint software limits (taken from the official
    SO-101 URDF, converted to servo ticks) and to a max step per command;
  * torque is never enabled implicitly, and is released on close();
  * temperature / voltage / load and servo error flags are checked before each
    move and the move is refused if anything is out of range.

Protocol: Feetech SCS/STS series, half-duplex TTL, 1 Mbaud, 4096 ticks / 360 deg.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

# ---------------------------------------------------------------- register map
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED = 46
ADDR_GOAL_ACCELERATION = 41
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED = 58
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_MOVING = 66
# calibration-related registers (see calib/arm_zero.py)
ADDR_PHASE = 18
ADDR_HOMING_OFFSET = 31
ADDR_LOCK = 55

TICKS_PER_TURN = 4096
DEG_PER_TICK = 360.0 / TICKS_PER_TURN

# STS3215 model number reported by ping
STS3215_MODEL = 777

# Bus transaction retry policy for READS.
#
# This is a half-duplex 1 Mbaud line with no flow control, and `read_all()` issues 36 separate
# transactions per call (6 joints x 6 registers). At even a 1% per-transaction loss the chance
# that a whole read_all() fails is about 30% -- which is exactly what the bench showed:
# intermittent "There is no status packet" faults, arriving right after a power-supply change
# and with the voltage a healthy 6.9 V, so they read as a wiring problem and were not one.
#
# Only TRANSPORT failures retry. A servo-reported ERROR (over-voltage, over-heat, stalled) is a
# fact about the hardware and must reach the caller on the very first read.
READ_ATTEMPTS = 4
READ_RETRY_SETTLE_S = 0.003

# Joint order as wired on the SO-101 follower (servo ID 1..6)
JOINT_IDS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

# URDF joint limits (deg) -> used as the software safety envelope.
# Source: docs/so101.urdf (Rotation/Pitch/Elbow/Wrist_Pitch/Wrist_Roll/Jaw).
# MARGIN is subtracted from both ends; keep it generous until the calibration
# offsets for THIS arm are known, because raw tick <-> joint-degree mapping is
# not yet established.
URDF_LIMITS_DEG = {
    "shoulder_pan": (69.9, 289.9),
    "shoulder_lift": (79.9, 280.0),
    "elbow_flex": (80.0, 270.0),
    "wrist_flex": (85.0, 275.0),
    "wrist_roll": (22.8, 342.8),
    "gripper": (180.0, 280.0),  # Jaw: 3.14..4.887 rad
}
SAFETY_MARGIN_DEG = 8.0


class SafetyError(RuntimeError):
    """Raised instead of moving when a guardrail would be violated."""


class ServoFault(RuntimeError):
    """Raised when the bus or a servo reports a hardware problem."""


@dataclass
class JointState:
    name: str
    servo_id: int
    ticks: int
    raw_deg: float
    voltage: float
    temperature: int
    torque_enabled: bool
    moving: bool
    error: int

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "servo_id": self.servo_id,
            "ticks": self.ticks,
            "raw_deg": round(self.raw_deg, 2),
            "voltage": self.voltage,
            "temperature": self.temperature,
            "torque_enabled": self.torque_enabled,
            "moving": self.moving,
            "error": self.error,
        }


@dataclass
class SafetyConfig:
    max_step_deg: float = 15.0          # per joint, per command_sequence step
    max_speed_ticks_s: int = 600        # STS3215 goal-speed register units
    accel_ticks_s2: int = 30
    limit_margin_deg: float = 6.0       # extra margin off the URDF joint limits
    min_voltage: float = 4.2            # a 6 V pack sags; below this, stop
    max_temperature: int = 65
    max_load_pct: float = 80.0
    settle_timeout_s: float = 6.0
    limits_deg: dict = field(default_factory=lambda: dict(URDF_LIMITS_DEG))


class ServoBus:
    """Context-managed, safety-guarded access to the SO-101 follower bus."""

    def __init__(
        self,
        port: str = "COM10",
        baudrate: int = 1_000_000,
        *,
        allow_motion: bool = False,
        safety: SafetyConfig | None = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.allow_motion = allow_motion
        self.safety = safety or SafetyConfig()
        self._port: PortHandler | None = None
        self._pkt = PacketHandler(0)
        self._torque_on: set[int] = set()

    # ------------------------------------------------------------- lifecycle
    def __enter__(self) -> "ServoBus":
        self._port = PortHandler(self.port)
        if not self._port.openPort():
            raise ServoFault(f"cannot open {self.port}")
        if not self._port.setBaudRate(self.baudrate):
            self._port.closePort()
            raise ServoFault(f"cannot set baudrate {self.baudrate} on {self.port}")
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def close(self) -> None:
        for sid in list(self._torque_on):
            try:
                self._write1(sid, ADDR_TORQUE_ENABLE, 0)
            except Exception:
                pass
        self._torque_on.clear()
        if self._port is not None:
            self._port.closePort()
            self._port = None

    # -------------------------------------------------------------- low level
    def _check(self, comm: int, err: int, what: str) -> None:
        if comm != COMM_SUCCESS:
            raise ServoFault(f"{what}: bus error {self._pkt.getTxRxResult(comm)}")
        if err:
            raise ServoFault(f"{what}: servo error {self._pkt.getRxPacketError(err)}")

    def _write1(self, sid: int, addr: int, value: int) -> None:
        comm, err = self._pkt.write1ByteTxRx(self._port, sid, addr, value)
        self._check(comm, err, f"write1 id={sid} addr={addr}")

    def _write2(self, sid: int, addr: int, value: int) -> None:
        comm, err = self._pkt.write2ByteTxRx(self._port, sid, addr, value)
        self._check(comm, err, f"write2 id={sid} addr={addr}")

    def _read_retry(self, label: str, txrx, sid: int, addr: int) -> int:
        """One read transaction, retried on a transport failure. See READ_ATTEMPTS.

        A transport success carrying a servo error is deliberately NOT retried: the hardware is
        reporting a fact, and reading it four times does not change it.
        """
        last = COMM_SUCCESS
        for attempt in range(READ_ATTEMPTS):
            value, comm, err = txrx(self._port, sid, addr)
            if comm == COMM_SUCCESS:
                self._check(comm, err, f"{label} id={sid} addr={addr}")
                return value
            last = comm
            if attempt + 1 < READ_ATTEMPTS:
                time.sleep(READ_RETRY_SETTLE_S)
        raise ServoFault(
            f"{label} id={sid} addr={addr}: bus error {self._pkt.getTxRxResult(last)} "
            f"(after {READ_ATTEMPTS} attempts)")

    def _read1(self, sid: int, addr: int) -> int:
        return self._read_retry("read1", self._pkt.read1ByteTxRx, sid, addr)

    def _read2(self, sid: int, addr: int) -> int:
        return self._read_retry("read2", self._pkt.read2ByteTxRx, sid, addr)

    # ------------------------------------------------------------- discovery
    def scan(self, id_range: range = range(1, 8)) -> dict[int, int]:
        found: dict[int, int] = {}
        for sid in id_range:
            model, comm, _err = self._pkt.ping(self._port, sid)
            if comm == COMM_SUCCESS and model != -1:
                found[sid] = model
        return found

    def read_all(self) -> dict[str, JointState]:
        states: dict[str, JointState] = {}
        for name, sid in JOINT_IDS.items():
            ticks = self._read2(sid, ADDR_PRESENT_POSITION)
            states[name] = JointState(
                name=name,
                servo_id=sid,
                ticks=ticks,
                raw_deg=ticks * DEG_PER_TICK,
                voltage=self._read1(sid, ADDR_PRESENT_VOLTAGE) / 10.0,
                temperature=self._read1(sid, ADDR_PRESENT_TEMPERATURE),
                torque_enabled=bool(self._read1(sid, ADDR_TORQUE_ENABLE)),
                moving=bool(self._read1(sid, ADDR_MOVING)),
                error=self._read1(sid, 65),  # Hardware error status
            )
        return states

    # ----------------------------------------------------------------- checks
    def _limit_ticks(self, name: str) -> tuple[int, int]:
        lo, hi = self.safety.limits_deg[name]
        lo += SAFETY_MARGIN_DEG
        hi -= SAFETY_MARGIN_DEG
        return int(round(lo / DEG_PER_TICK)), int(round(hi / DEG_PER_TICK))

    def preflight(self) -> list[str]:
        """Return a list of blocking problems; empty list means safe to energise."""
        problems: list[str] = []
        states = self.read_all()
        for st in states.values():
            if st.error:
                problems.append(f"{st.name}: servo error flag {st.error}")
            if st.temperature >= self.safety.max_temperature:
                problems.append(f"{st.name}: {st.temperature} C >= limit")
            if st.voltage < self.safety.min_voltage:
                problems.append(f"{st.name}: {st.voltage:.1f} V < {self.safety.min_voltage} V")
        return problems

    # ---------------------------------------------------------------- motion
    def enable_torque(self, force: bool = False) -> None:
        if not self.allow_motion and not force:
            raise SafetyError("ServoBus built with allow_motion=False")
        problems = self.preflight()
        if problems:
            raise SafetyError("preflight failed: " + "; ".join(problems))
        for name, sid in JOINT_IDS.items():
            self._write1(sid, ADDR_TORQUE_ENABLE, 1)
            self._torque_on.add(sid)

    def disable_torque(self) -> None:
        for sid in list(JOINT_IDS.values()):
            try:
                self._write1(sid, ADDR_TORQUE_ENABLE, 0)
            finally:
                self._torque_on.discard(sid)

    def set_goal_ticks(self, goals: dict[str, int], speed: int | None = None) -> None:
        """Write goal positions once, after clamping each to the joint envelope."""
        if not self.allow_motion:
            raise SafetyError("motion disabled (allow_motion=False)")
        speed = self.safety.max_speed_ticks_s if speed is None else min(speed, self.safety.max_speed_ticks_s)
        current = self.read_all()
        for name, goal in goals.items():
            lo, hi = self._limit_ticks(name)
            clamped = max(lo, min(hi, int(goal)))
            if clamped != int(goal):
                raise SafetyError(
                    f"{name}: goal {goal} ticks outside envelope [{lo}, {hi}]; refusing to move"
                )
            delta_deg = abs(clamped - current[name].ticks) * DEG_PER_TICK
            if delta_deg > self.safety.max_step_deg:
                raise SafetyError(
                    f"{name}: step {delta_deg:.1f} deg > max_step_deg {self.safety.max_step_deg}"
                )
            sid = JOINT_IDS[name]
            self._write2(sid, ADDR_GOAL_SPEED, speed)
            self._write2(sid, ADDR_GOAL_POSITION, clamped)

    def wait_until_settled(self, timeout_s: float | None = None) -> bool:
        timeout_s = self.safety.settle_timeout_s if timeout_s is None else timeout_s
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if not any(self._read1(sid, ADDR_MOVING) for sid in JOINT_IDS.values()):
                return True
            time.sleep(0.05)
        return False

    def hold_current(self) -> None:
        """Freeze the arm where it is (used before any scripted motion)."""
        current = self.read_all()
        goals = {n: st.ticks for n, st in current.items()}
        for name, goal in goals.items():
            sid = JOINT_IDS[name]
            self._write2(sid, ADDR_GOAL_SPEED, self.safety.max_speed_ticks_s)
            self._write2(sid, ADDR_GOAL_POSITION, goal)


def main() -> None:
    import argparse
    import json

    ap = argparse.ArgumentParser(description="SO-101 read-only status probe")
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    with ServoBus(args.port, allow_motion=False) as bus:
        found = bus.scan()
        states = bus.read_all()
        problems = bus.preflight()
        if args.json:
            print(json.dumps(
                {
                    "port": args.port,
                    "found": {str(k): v for k, v in found.items()},
                    "joints": {n: s.as_dict() for n, s in states.items()},
                    "problems": problems,
                },
                indent=2,
            ))
            return
        print(f"bus {args.port} @1Mbaud  found ids: {sorted(found)}")
        print(f"{'joint':<15}{'id':<4}{'ticks':<7}{'raw_deg':<9}{'V':<6}{'T(C)':<6}{'torque':<8}{'moving'}")
        for s in states.values():
            print(f"{s.name:<15}{s.servo_id:<4}{s.ticks:<7}{s.raw_deg:<9.1f}{s.voltage:<6.1f}"
                  f"{s.temperature:<6}{str(s.torque_enabled):<8}{s.moving}")
        print("preflight:", problems or "no blocking problems")


if __name__ == "__main__":
    main()

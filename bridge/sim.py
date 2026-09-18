"""A simulated SO-101 plant, so the whole pipeline can be exercised without hardware.

Why this exists
---------------
Joint-zero calibration needs a human to hold the arm at its range midpoint, so hardware
motion is blocked on that.  Everything ABOVE the servo bus, however, is not: perception,
target selection, IK, rate limiting, the safety envelope, the HTTP bridge, and the DSH
tool wiring are all testable today.  A simulation backend lets the agent drive a full
"look -> decide -> move -> grasp -> verify" loop and assert on it, which de-risks the
whole stack before the arm is ever allowed to move.

Design
------
`SimServoBus` deliberately mirrors `ServoBus`'s private register surface
(`_read1`/`_read2`/`_write1`/`_write2`/`scan`/`read_all`/`preflight`/`enable_torque`/
`disable_torque`/`wait_until_settled`).  It is not a stub that pretends to work: it
integrates the servo trajectory generator the same way the hardware does, honours
Goal_Speed and Goal_Acceleration, applies the servo's own acceleration limit, and
reports present position back.  So a motion that the real arm would refuse (joint limit,
rate limit, envelope) is refused here too, for the same reasons.

It also models the two things that actually bite in practice:

  * a gripper that closes on nothing travels its full range, while a gripper that closes
    on an object stops early -- which is how the agent verifies a grasp at all;
  * electrical load: holding torque against a stop draws current and heats the servo.

The 1 Mbaud bus is not simulated: reads return immediately.  A caller that depends on
serial latency is not testing anything real anyway.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

from bridge.servo_bus import (
    ADDR_GOAL_ACCELERATION,
    ADDR_GOAL_POSITION,
    ADDR_GOAL_SPEED,
    ADDR_HOMING_OFFSET,
    ADDR_LOCK,
    ADDR_MOVING,
    ADDR_PHASE,
    ADDR_PRESENT_LOAD,
    ADDR_PRESENT_POSITION,
    ADDR_PRESENT_SPEED,
    ADDR_PRESENT_TEMPERATURE,
    ADDR_PRESENT_VOLTAGE,
    ADDR_TORQUE_ENABLE,
    JOINT_IDS,
    STS3215_MODEL,
    TICKS_PER_TURN,
    JointState,
    ServoFault,
    DEG_PER_TICK,
)


@dataclass
class SimConfig:
    """Plant parameters.  Defaults are a plausible healthy SO-101."""

    voltage: float = 4.6
    temperature: float = 30.0
    #: ticks/s the servo can physically do, independent of the Goal_Speed register
    max_speed_ticks_s: float = 1200.0
    #: how much the servo overshoots/settles; 0 means it stops exactly on the goal
    noise_ticks: float = 0.0
    #: ticks of error that persist after every move, i.e. a repeatability error
    repeatability_ticks: float = 0.0
    #: thermal rise per second while loaded, and cooling per second at rest
    heat_per_s_loaded: float = 0.4
    cool_per_s_idle: float = 0.15
    #: object present in the jaw: (joint name, tick width the gripper closes to)
    grasped_width_ticks: dict | None = None
    seed: int = 0


class SimServoBus:
    """In-memory stand-in for the Feetech bus, with a real trajectory generator."""

    def __init__(
        self,
        port: str = "SIM",
        baudrate: int = 1_000_000,
        *,
        allow_motion: bool = False,
        safety=None,
        config: SimConfig | None = None,
        initial_ticks: dict[str, int] | None = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.allow_motion = allow_motion
        self.cfg = config or SimConfig()
        self._safety = safety
        self._closed = False

        # a plausible resting pose: the values the real arm reports when it is sitting
        # on the bench with torque off
        self._ticks: dict[int, int] = {
            JOINT_IDS[n]: v for n, v in (initial_ticks or {
                "shoulder_pan": 2063, "shoulder_lift": 953, "elbow_flex": 2994,
                "wrist_flex": 2805, "wrist_roll": 1978, "gripper": 1976,
            }).items()
        }
        self._goal: dict[int, int] = dict(self._ticks)
        self._speed: dict[int, int] = {sid: 600 for sid in JOINT_IDS.values()}
        self._accel: dict[int, int] = {sid: 30 for sid in JOINT_IDS.values()}
        self._torque: dict[int, bool] = {sid: False for sid in JOINT_IDS.values()}
        self._moving: dict[int, bool] = {sid: False for sid in JOINT_IDS.values()}
        self._load: dict[int, int] = {sid: 0 for sid in JOINT_IDS.values()}
        self._temp: dict[int, float] = {sid: self.cfg.temperature for sid in JOINT_IDS.values()}
        self._homing: dict[int, int] = {sid: 0 for sid in JOINT_IDS.values()}
        self._phase: dict[int, int] = {sid: 0 for sid in JOINT_IDS.values()}
        self._locked: dict[int, bool] = {sid: False for sid in JOINT_IDS.values()}
        self._last_t: float = time.time()
        self._reference_width: dict[int, int] = {}
        self._in_plant = False
        self.plant = None                     # a SimScene, attached by bridge/sim_scene.py
        self.events: list[dict] = []

    # ------------------------------------------------------------- lifecycle
    def __enter__(self) -> "SimServoBus":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        self._closed = True

    def _advance(self) -> None:
        """Integrate the trajectory generator forward to 'now'."""
        now = time.time()
        dt = max(0.0, now - self._last_t)
        self._last_t = now
        if dt <= 0:
            return
        for sid in JOINT_IDS.values():
            if not self._torque[sid]:
                self._moving[sid] = False
                self._load[sid] = 0
                self._temp[sid] = max(self.cfg.temperature,
                                      self._temp[sid] - self.cfg.cool_per_s_idle * dt)
                continue
            cur = self._ticks[sid]
            goal = self._goal[sid]
            err = goal - cur
            if err == 0:
                self._moving[sid] = False
                self._load[sid] = int(min(100, abs(self._load[sid]) * 0.5))
                self._temp[sid] = max(self.cfg.temperature,
                                      self._temp[sid] - self.cfg.cool_per_s_idle * dt)
                continue
            # speed is limited by both the register and the physical capability
            v = min(self._speed[sid], self.cfg.max_speed_ticks_s)
            step = min(v * dt, abs(err))
            self._ticks[sid] = int(cur + math.copysign(round(step), err))
            if self._ticks[sid] != cur:
                self._moving[sid] = True
                load = min(95, int(100 * abs(err) / 200 + 15))
                self._load[sid] = load
                self._temp[sid] += self.cfg.heat_per_s_loaded * dt * (load / 50.0)

        # An object between the fingers is a hard travel limit.  Applied here, and also
        # immediately after a goal write (see _apply_gripper_limit), because a goal that is
        # already past the obstruction would otherwise let the plant integrate past it.
        self._apply_gripper_limit()

        # The world (bridge/sim_scene.py) rides this same tick, so a grasp is decided from
        # the jaw position the bus actually integrated to rather than from whatever instant
        # a caller happened to sample.  Physics in the integration loop, not in a caller's
        # diligence: the motion layer closes the jaws over several rate-limited commands
        # inside one tool call, so a hand-invoked sync arrives too late and the jaw sails
        # straight through the block.
        #
        # The recursion guard is load-bearing.  The plant reads registers to work out what
        # is between the fingers, and every register read integrates; without the flag
        # _advance -> plant.step -> _read2 -> _advance would never terminate.  A nested call
        # is at the same instant with nothing left to integrate, so it is simply skipped.
        if self.plant is not None and not self._in_plant:
            self._in_plant = True
            try:
                self.plant.step()
            finally:
                self._in_plant = False

    def _apply_gripper_limit(self) -> None:
        """Hold the jaw at the object width when it is being driven to close further.

        Closing is toward LARGER tick values, so "is it trying to close past the object?" is
        a plain directed comparison against the width.

        The first version compared |goal - mid| against |limit - mid|, which cannot tell the
        two ends of the travel apart: the jaw is very nearly symmetric about the midpoint
        (open 1537, shut 2617, midpoint 2048), so a fully OPEN command sat further from the
        midpoint than the object's width did.  The clamp therefore fired while the jaws were
        opening, pinned them shut and never let the object go -- a release that reported
        success while the object stayed in the fingers.
        """
        block = self.cfg.grasped_width_ticks
        if not block:
            return
        gsid = JOINT_IDS["gripper"]
        if not self._torque[gsid]:
            return
        limit = int(block["gripper"])
        cur = int(self._ticks[gsid])
        goal = int(self._goal[gsid])
        if goal > limit and cur > limit:
            self._ticks[gsid] = limit
            self._goal[gsid] = limit
            self._moving[gsid] = False
            self._load[gsid] = 70

    # -------------------------------------------------------- register access
    def scan(self, id_range=range(1, 8)) -> dict[int, int]:
        return {sid: STS3215_MODEL for sid in JOINT_IDS.values() if sid in id_range}

    def read_all(self) -> dict[str, JointState]:
        self._advance()
        out: dict[str, JointState] = {}
        for name, sid in JOINT_IDS.items():
            t = self._ticks[sid]
            out[name] = JointState(
                name=name, servo_id=sid, ticks=t, raw_deg=t * DEG_PER_TICK,
                voltage=self.cfg.voltage, temperature=int(round(self._temp[sid])),
                torque_enabled=self._torque[sid], moving=self._moving[sid], error=0,
            )
        return out

    def preflight(self) -> list[str]:
        problems: list[str] = []
        for name, sid in JOINT_IDS.items():
            if self._temp[sid] >= 65:
                problems.append(f"{name}: {self._temp[sid]:.0f} C >= limit")
            if self.cfg.voltage < 4.2:
                problems.append(f"{name}: {self.cfg.voltage:.1f} V < 4.2 V")
        return problems

    def hold_current(self) -> None:
        """Freeze the arm where it is.  Same contract as ServoBus.hold_current.

        The real bus needs this called explicitly before enabling torque, because
        Torque_Enable=1 drives each servo toward whatever Goal_Position already holds -- and
        after a watchdog release that register is a stale pose, so the arm snaps.  This
        simulator has always synced the goal inside enable_torque instead, so the driver got
        away with omitting it here and whipped the real arm at 24 deg.  Keeping the two buses
        on the same contract is the whole point of driving the production driver in the test.
        """
        for sid in JOINT_IDS.values():
            self._goal[sid] = self._ticks[sid]

    def enable_torque(self, force: bool = False) -> None:
        for sid in JOINT_IDS.values():
            self._torque[sid] = True
            self._goal[sid] = self._ticks[sid]
        self.events.append({"t": time.time(), "event": "torque_on"})

    def disable_torque(self) -> None:
        for sid in JOINT_IDS.values():
            self._torque[sid] = False
        self.events.append({"t": time.time(), "event": "torque_off"})

    def wait_until_settled(self, timeout_s: float = 6.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self._advance()
            if not any(self._moving[sid] for sid in JOINT_IDS.values()):
                return True
            time.sleep(0.01)
        return False

    def _write1(self, sid: int, addr: int, value: int) -> None:
        self._require(sid)
        if addr == ADDR_TORQUE_ENABLE:
            self._torque[sid] = bool(value)
            if value:
                self._goal[sid] = self._ticks[sid]
        elif addr == ADDR_GOAL_ACCELERATION:
            self._accel[sid] = value
        elif addr == ADDR_PRESENT_TEMPERATURE:
            raise ServoFault("temperature is read-only")
        elif addr == ADDR_HOMING_OFFSET:
            self._homing[sid] = value
        elif addr == ADDR_PHASE:
            self._phase[sid] = value
        elif addr == ADDR_LOCK:
            self._locked[sid] = bool(value)
        else:
            raise ServoFault(f"sim: unsupported 1-byte write to register {addr}")

    def _write2(self, sid: int, addr: int, value: int) -> None:
        self._require(sid)
        if addr == ADDR_GOAL_POSITION:
            if not self._torque[sid]:
                raise ServoFault(f"sim: goal written to servo {sid} with torque off")
            self._goal[sid] = value % TICKS_PER_TURN
            # A servo sets its Moving flag as soon as it is given a new goal, not once it
            # has started integrating.  Without this the plant looks settled the instant a
            # goal is written, `wait_until_settled` returns immediately, and the arm never
            # moves at all -- which is exactly how this behaved before the fix.
            self._moving[sid] = self._goal[sid] != self._ticks[sid]
            self._last_t = time.time()
            self._apply_gripper_limit()
        elif addr == ADDR_GOAL_SPEED:
            self._speed[sid] = value
        elif addr == ADDR_HOMING_OFFSET:
            self._homing[sid] = value
        else:
            raise ServoFault(f"sim: unsupported 2-byte write to register {addr}")

    def _read1(self, sid: int, addr: int) -> int:
        self._require(sid)
        self._advance()
        if addr == ADDR_TORQUE_ENABLE:
            return int(self._torque[sid])
        if addr == ADDR_PRESENT_VOLTAGE:
            return int(round(self.cfg.voltage * 10))
        if addr == ADDR_PRESENT_TEMPERATURE:
            return int(round(self._temp[sid]))
        if addr == ADDR_PRESENT_LOAD:
            return int(self._load[sid])
        if addr == ADDR_MOVING:
            return int(self._moving[sid])
        if addr == ADDR_PHASE:
            return self._phase[sid]
        if addr == ADDR_HOMING_OFFSET:
            return self._homing[sid]
        if addr == 65:                       # hardware error status
            return 0
        raise ServoFault(f"sim: unsupported 1-byte read from register {addr}")

    def _read2(self, sid: int, addr: int) -> int:
        self._require(sid)
        self._advance()
        if addr == ADDR_PRESENT_POSITION:
            return self._ticks[sid]
        if addr == ADDR_PRESENT_SPEED:
            return self._speed[sid]
        if addr == ADDR_HOMING_OFFSET:
            return self._homing[sid]
        raise ServoFault(f"sim: unsupported 2-byte read from register {addr}")

    def _require(self, sid: int) -> None:
        if self._closed:
            raise ServoFault("sim: bus is closed")
        if sid not in self._ticks:
            raise ServoFault(f"sim: no servo with id {sid}")

    # ------------------------------------------------------------- test hooks
    def place_object_in_gripper(self, width_ticks: int) -> None:
        """Make the gripper stop at `width_ticks` instead of running to its goal.

        This is exactly how a grasp is detected in the real system: a gripper that closes
        on nothing travels its whole range, one that closes on an object stalls early.
        """
        self.cfg.grasped_width_ticks = {"gripper": width_ticks}

    def move_joint_forcibly(self, name: str, ticks: int) -> None:
        """Back-drive a joint with torque off (simulates a human positioning the arm)."""
        sid = JOINT_IDS[name]
        self._torque[sid] = False
        self._ticks[sid] = ticks % TICKS_PER_TURN
        self._goal[sid] = self._ticks[sid]

    def snapshot(self) -> dict[str, int]:
        self._advance()
        return {name: self._ticks[sid] for name, sid in JOINT_IDS.items()}


def sim_calibration(
    kin,
    middle_ticks: dict[str, int] | None = None,
    direction_sign: dict[str, int] | None = None,
) -> dict:
    """A calibration file that is EXACT for the simulation.

    The simulation works in raw ticks, so the tick<->degree conversion still goes through
    the real `ticks_to_deg` code path.  This builds a calibration whose middle pose is
    known exactly, which means the loop below genuinely exercises FK, IK, the safety
    envelope and the conversion -- instead of testing a self-consistent fiction.

    `direction_sign` is the one thing that cannot be derived: on real hardware it is a
    property of how each servo was assembled, and it is discovered by the joint-zero
    calibration.  In the simulation the true sign is +1 by construction, so the loop
    verifies the rest of the chain and leaves that one bit to the real calibration.
    """
    from bridge.kinematics import LEROBOT_TO_URDF

    mid_deg = {}
    for lname, urdf_name in LEROBOT_TO_URDF.items():
        lo, hi = kin.limits_deg[urdf_name]
        mid_deg[lname] = (lo + hi) / 2.0
    ticks = middle_ticks or {
        "shoulder_pan": 2048, "shoulder_lift": 2048, "elbow_flex": 2048,
        "wrist_flex": 2048, "wrist_roll": 2048, "gripper": 2048,
    }
    return {
        "format": "ds101-so101-calibration",
        "version": 1,
        "created": "simulation",
        "port": "SIM",
        "ticks_per_turn": TICKS_PER_TURN,
        "middle_ticks": {k: int(v) for k, v in ticks.items()},
        "direction_sign": direction_sign or {k: 1 for k in JOINT_IDS},
        "joint_limits_deg": {k: list(kin.limits_deg[v]) for k, v in LEROBOT_TO_URDF.items()},
        "middle_pose_deg": mid_deg,
        "homing_written": False,
        "notes": "generated by bridge/sim.py; exact by construction, not a real calibration",
    }


def degrees_to_ticks_exact(deg: dict[str, float], kin) -> dict[str, int]:
    """Convert true joint degrees to the ticks a `sim_calibration` arm would report.

    Used only to seed a simulation at a known true pose (so the starting tool position is
    known independently of the driver's own conversion).
    """
    from bridge.kinematics import LEROBOT_TO_URDF

    out: dict[str, int] = {}
    for lname, urdf_name in LEROBOT_TO_URDF.items():
        lo, hi = kin.limits_deg[urdf_name]
        mid = (lo + hi) / 2.0
        out[lname] = int(round(2048 + (deg[lname] - mid) * TICKS_PER_TURN / 360.0))
    return out

"""The local motion layer: everything between "the agent decided X" and the servos.

Division of labour, and why it is drawn here
--------------------------------------------
The agent (a language model) is a 0.3-1 Hz planner.  It must never be in the servo
loop: a model round-trip is seconds, a servo stalls in milliseconds.  So this module
owns everything time-critical:

  * tick <-> joint-degree conversion via the calibration file,
  * clamping to the URDF joint envelope with a margin,
  * using the STS3215's own trajectory generator (Goal_Speed + Goal_Acceleration) so
    interpolation happens in the servo at ~50-100 Hz, not over the serial link,
  * a heartbeat watchdog: if the caller stops checking in, torque is released,
  * a Cartesian entry point that runs IK and refuses to move on a bad solution.

The watchdog matters because there is no hardware e-stop yet.  It is not a substitute
for one -- a physical switch on the servo power is still required -- but it does stop
the common failure mode where software hangs with torque on and the arm holds a
stall current until something melts.
"""
from __future__ import annotations

import json
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# allow running this file directly (python bridge/driver.py) from the project root
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bridge.kinematics import LEROBOT_TO_URDF, SO101Kinematics  # noqa: E402
from bridge.servo_bus import (
    ADDR_GOAL_ACCELERATION,
    ADDR_GOAL_POSITION,
    ADDR_GOAL_SPEED,
    ADDR_PRESENT_POSITION,
    ADDR_PRESENT_TEMPERATURE,
    ADDR_PRESENT_VOLTAGE,
    ADDR_TORQUE_ENABLE,
    JOINT_IDS,
    SafetyConfig,
    SafetyError,
    ServoBus,
    TICKS_PER_TURN,
)


def load_calibration(path: str | Path) -> dict:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(
            f"no calibration at {p}. Run calib/arm_zero.py first: without it, raw ticks "
            "are not joint angles and every FK/IK result is meaningless."
        )
    return json.loads(p.read_text(encoding="utf-8"))


def ticks_to_deg(ticks: dict[str, int], calib: dict) -> dict[str, float]:
    out: dict[str, float] = {}
    for lname, mid_tick in calib["middle_ticks"].items():
        sign = calib["direction_sign"].get(lname, 1)
        delta = ticks[lname] - int(mid_tick)
        out[lname] = calib["middle_pose_deg"][lname] + delta * 360.0 / TICKS_PER_TURN * sign
    return out


def deg_to_ticks(deg: dict[str, float], calib: dict) -> dict[str, int]:
    out: dict[str, int] = {}
    for lname, value in deg.items():
        mid_tick = int(calib["middle_ticks"][lname])
        sign = calib["direction_sign"].get(lname, 1)
        delta = (value - calib["middle_pose_deg"][lname]) / sign
        out[lname] = int(round(mid_tick + delta * TICKS_PER_TURN / 360.0))
    return out


GRIPPER_CLOSED_DEG = 280.0
GRIPPER_OPEN_DEG = 185.0


@dataclass
class WatchdogConfig:
    enabled: bool = True
    timeout_s: float = 5.0
    poll_s: float = 0.25


class ArmDriver:
    """Single-owner, safety-guarded controller for the SO-101 follower."""

    def __init__(
        self,
        port: str = "COM10",
        calibration_path: str | Path = "calib/out/so101_calibration.json",
        *,
        allow_motion: bool = False,
        urdf_path: str | Path | None = None,
        safety: SafetyConfig | None = None,
        watchdog: WatchdogConfig | None = None,
        calibration: dict | None = None,
        bus=None,
    ) -> None:
        """`calibration` overrides the file, and `bus` overrides the serial bus.

        Both exist so the whole driver -- tick<->degree conversion, rate limiting, joint
        envelope clamping, the trajectory generator, the watchdog -- can be exercised
        against an in-memory plant (see `bridge/sim.py`) instead of the real servos. The
        simulation therefore tests the production code path rather than a parallel one.
        """
        self.calib = calibration if calibration is not None else load_calibration(calibration_path)
        self.kin = SO101Kinematics(urdf_path) if urdf_path else SO101Kinematics()
        self.safety = safety or SafetyConfig()
        self.watchdog_cfg = watchdog or WatchdogConfig()
        self.bus = bus if bus is not None else ServoBus(port, allow_motion=allow_motion, safety=self.safety)
        self.allow_motion = allow_motion
        self._last_heartbeat = time.time()
        self._torque_on = False
        self._watchdog_thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.RLock()
        self._log: list[dict] = []

    # ------------------------------------------------------------- lifecycle
    def connect(self) -> "ArmDriver":
        self.bus.__enter__()
        found = self.bus.scan()
        missing = sorted(set(JOINT_IDS.values()) - set(found))
        if missing:
            self.bus.close()
            raise RuntimeError(f"servo ids {missing} not responding on {self.bus.port}")
        if self.watchdog_cfg.enabled and self.allow_motion:
            self._watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
            self._watchdog_thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        if self._watchdog_thread is not None:
            self._watchdog_thread.join(timeout=2.0)
        self.release()
        self.bus.close()

    def __enter__(self) -> "ArmDriver":
        return self.connect()

    def __exit__(self, *exc) -> None:
        self.close()

    # -------------------------------------------------------------- readback
    def state(self) -> dict:
        st = self.bus.read_all()
        raw = {name: s.ticks for name, s in st.items()}
        deg = ticks_to_deg(raw, self.calib)
        tool = None
        try:
            tool = self.kin.fk({LEROBOT_TO_URDF[k]: v for k, v in deg.items() if k in LEROBOT_TO_URDF})
        except Exception:
            pass
        return {
            "raw_ticks": raw,
            "joint_deg": {k: round(v, 2) for k, v in deg.items()},
            "tool_mm": None if tool is None else [round(float(x) * 1000, 1) for x in tool[:3, 3]],
            "voltage": {n: s.voltage for n, s in st.items()},
            "temperature": {n: s.temperature for n, s in st.items()},
            "torque_enabled": self._torque_on,
            "motion_allowed": self.allow_motion,
        }

    # ------------------------------------------------------------ heartbeat
    def heartbeat(self) -> None:
        """Call from the client loop; silence beyond the timeout releases torque."""
        self._last_heartbeat = time.time()

    def _watchdog_loop(self) -> None:
        while not self._stop.wait(self.watchdog_cfg.poll_s):
            if not self._torque_on:
                continue
            if time.time() - self._last_heartbeat > self.watchdog_cfg.timeout_s:
                self._log.append({"t": time.time(), "event": "watchdog_release"})
                try:
                    self.bus.disable_torque()
                finally:
                    self._torque_on = False

    # --------------------------------------------------------------- motion
    def enable(self) -> None:
        if not self.allow_motion:
            raise SafetyError("driver built with allow_motion=False")
        problems = self.bus.preflight()
        if problems:
            raise SafetyError("preflight failed: " + "; ".join(problems))
        # Torque_Enable=1 makes each servo drive toward the Goal_Position already sitting
        # in its register.  That register holds the last *commanded* pose, and after a
        # watchdog release or an estop the arm has sagged somewhere else entirely, so
        # enabling directly snaps every joint back toward a stale goal at full servo
        # speed -- measured here at 24 deg on elbow_flex, which is a whip, not a move.
        # Freezing the present pose first turns torque-on into a hold.
        self.bus.hold_current()
        self.bus.enable_torque()
        self._torque_on = True
        self.heartbeat()

    def release(self) -> None:
        try:
            self.bus.disable_torque()
        except Exception:
            pass
        self._torque_on = False

    def _envelope_deg(self, lname: str, urdf_name: str) -> tuple[float, float]:
        """The range this joint may be commanded into.

        MEASURED limits from the calibration file win over the URDF design limits. The URDF
        describes the arm the CAD file was drawn for; the calibration describes the arm on THIS
        bench, and the two disagree in the direction that matters: shoulder_lift was measured at
        210.8 deg of travel against a 200 deg design, so the design envelope is NARROWER than
        the real one. Clamping to it put the operator's own recorded HOME pose outside the
        envelope, and `arm_home` could only answer "target outside the joint envelope; the arm
        moved as far as allowed" -- the firmware refusing to reach a pose its operator recorded.
        """
        measured = (self.calib.get("joint_limits_deg") or {}).get(lname)
        if isinstance(measured, (list, tuple)) and len(measured) == 2:
            return float(measured[0]), float(measured[1])
        return self.kin.limits_deg[urdf_name]

    def _clamp_deg(self, q: dict[str, float]) -> tuple[dict[str, float], list[str]]:
        """Clamp to the joint envelope minus a margin; report what had to be clamped."""
        clamped: dict[str, float] = {}
        clipped: list[str] = []
        for lname, urdf_name in LEROBOT_TO_URDF.items():
            if lname not in q:
                continue
            lo, hi = self._envelope_deg(lname, urdf_name)
            lo += self.safety.limit_margin_deg
            hi -= self.safety.limit_margin_deg
            v = float(q[lname])
            if v < lo or v > hi:
                clipped.append(f"{lname}:{v:.1f}->{max(lo, min(hi, v)):.1f}")
                v = max(lo, min(hi, v))
            clamped[lname] = v
        return clamped, clipped

    def goto_joints(
        self,
        target_deg: dict[str, float],
        *,
        speed_ticks_s: int | None = None,
        wait: bool = True,
        use_acceleration: bool = True,
        max_commands: int = 24,
        tol_deg: float = 1.5,
    ) -> dict:
        """Move to a joint-space target using the servos' own trajectory generator.

        `max_commands` matters more than it looks.  Each individual command is rate-limited
        to `max_step_deg` (15 deg by default), so a large move needs several commands --
        and an earlier version of this function issued exactly one, returned `ok: true`,
        and left the arm part-way to the target.  A caller that trusts `ok` would then
        believe the arm had arrived.  This now iterates until the joints are within
        `tol_deg` of the target, the envelope/clamping stops progress, or the command
        budget is exhausted, and reports which of those happened.

        The return value distinguishes the three cases explicitly:
            status: "reached" | "clamped" | "budget_exhausted"
        """
        with self._lock:
            if not self._torque_on:
                self.enable()
            self.heartbeat()

            clamped, clipped = self._clamp_deg(target_deg)
            targets = dict(clamped)
            limited: list[str] = []
            commands = 0
            status = "budget_exhausted"
            settled_all = True
            stalled = 0
            previous: dict[str, float] | None = None

            for _ in range(max(1, max_commands)):
                # Renew the lease on every iteration.
                #
                # A move is not instantaneous: a large reconfiguration takes 11 commands and
                # well over 20 s, and the watchdog timeout is 5 s.  Renewing only once at the
                # top means the lease expires *mid-move* -- torque is released while the loop
                # is still writing goals, the last few commands drive a limp arm, and the
                # move reports a 2.5-6.7 deg shortfall that looks exactly like a position
                # servo losing to gravity.  It is not: the arm was unpowered.  A move that is
                # actively running is itself evidence the caller is present.
                self.heartbeat()
                raw = {n: st.ticks for n, st in self.bus.read_all().items()}
                now = ticks_to_deg(raw, self.calib)

                # Stop as soon as the arm stops making progress.
                #
                # Without this the loop burns its entire 24-command budget re-sending a target
                # the servos are not moving to, and then reports "budget_exhausted" -- which
                # reads as "the bridge gave up" rather than "this joint is holding a load".
                # It is the latter: elbow_flex carries the whole forearm, and a position servo
                # under gravity settles a couple of degrees short of its target. Measured at
                # 2.6 deg on this bench, stable across runs. Re-issuing the command cannot fix
                # that, so the honest thing is to stop and say so.
                if previous is not None and all(
                        abs(now[k] - previous[k]) <= 0.2 for k in now):
                    stalled += 1
                    if stalled >= 2:
                        status = "stalled"
                        break
                else:
                    stalled = 0
                previous = now

                # rate-limit each joint against the CURRENT read: the servo may still be
                # in flight, and commanding from a stale position would overshoot
                step_cmd: dict[str, float] = {}
                worst = 0.0
                for lname, value in targets.items():
                    delta = value - now[lname]
                    worst = max(worst, abs(delta))
                    if abs(delta) > self.safety.max_step_deg:
                        step_cmd[lname] = now[lname] + math.copysign(
                            self.safety.max_step_deg, delta)
                        note = f"{lname}:->{step_cmd[lname]:.1f} (of {value:.1f})"
                        if note not in limited:
                            limited.append(note)
                    else:
                        step_cmd[lname] = value
                if worst <= tol_deg:
                    status = "reached"
                    break

                goals = deg_to_ticks(step_cmd, self.calib)
                speed = speed_ticks_s if speed_ticks_s else self.safety.max_speed_ticks_s
                speed = min(speed, self.safety.max_speed_ticks_s)
                for lname, tick in goals.items():
                    sid = JOINT_IDS[lname]
                    self.bus._write2(sid, ADDR_GOAL_SPEED, speed)
                    if use_acceleration:
                        self.bus._write1(sid, ADDR_GOAL_ACCELERATION, self.safety.accel_ticks_s2)
                    self.bus._write2(sid, ADDR_GOAL_POSITION, tick % TICKS_PER_TURN)
                commands += 1

                settled = self.bus.wait_until_settled() if wait else True
                settled_all = settled_all and bool(settled)
                if wait and not settled:
                    status = "did_not_settle"
                    break

            final = ticks_to_deg({n: st.ticks for n, st in self.bus.read_all().items()}, self.calib)
            residual = {k: round(targets[k] - final[k], 2) for k in targets}
            worst_residual = max((abs(v) for v in residual.values()), default=0.0)
            if status == "budget_exhausted" and clipped:
                status = "clamped"
            if status == "budget_exhausted" and worst_residual > tol_deg:
                status = "budget_exhausted"

            event = {"t": time.time(), "event": "goto_joints", "target": clamped,
                     "clipped": clipped, "rate_limited": limited, "status": status,
                     "commands": commands}
            self._log.append(event)

            ok = status == "reached"
            return {
                "ok": ok,
                "status": status,
                "commands_issued": commands,
                "command_deg": {k: round(v, 2) for k, v in targets.items()},
                "final_deg": {k: round(v, 2) for k, v in final.items() if k in targets},
                "residual_deg": residual,
                "reached_within_deg": tol_deg,
                "clipped": clipped,
                "rate_limited": limited,
                "settled": settled_all,
                "state": self.state() if wait else None,
                "reason": None if ok else (
                    "target outside the joint envelope; the arm moved as far as allowed"
                    if status == "clamped" else
                    "command budget exhausted before reaching the target; call again"
                    if status == "budget_exhausted" else
                    f"stopped making progress {worst_residual:.1f} deg short; the remaining "
                    f"joint(s) are holding a load, and re-commanding will not move them"
                    if status == "stalled" else status),
            }

    def goto_tool(self, xyz_mm, *, seed: dict[str, float] | None = None, **kw) -> dict:
        """Cartesian entry point: IK, then move.  Refuses to move on a bad IK result."""
        target = np.asarray(xyz_mm, dtype=float).reshape(3) / 1000.0
        if seed is None:
            cur = ticks_to_deg({n: st.ticks for n, st in self.bus.read_all().items()}, self.calib)
            seed = {LEROBOT_TO_URDF[k]: v for k, v in cur.items() if k in LEROBOT_TO_URDF}
        res = self.kin.ik(target, seed)
        if not res["ok"]:
            return {"ok": False, "reason": f"ik_failed:{res['reason']}",
                    "residual_mm": res["residual_mm"], "ik": res}
        out = self.goto_joints(res["q_deg"], **kw)
        out["ik"] = {"residual_mm": res["residual_mm"], "iters": res["iters"]}
        return out

    def set_gripper(self, openness_pct: float, **kw) -> dict:
        """0 % = closed, 100 % = open.

        Goes through the same multi-command, rate-limited path as every other joint
        (`goto_joints`), which matters for grasp detection: a single rate-limited command
        only moves the jaw 15 deg, so the jaw never reaches either the object or its own
        closed limit and the "did it stall?" signal becomes meaningless.
        """
        pct = max(0.0, min(100.0, float(openness_pct)))
        deg = GRIPPER_CLOSED_DEG + (GRIPPER_OPEN_DEG - GRIPPER_CLOSED_DEG) * (pct / 100.0)
        res = self.goto_joints({"gripper": deg}, **kw)
        res["openness_pct"] = pct
        res["commanded_deg"] = round(deg, 2)
        res["measured_deg"] = res.get("final_deg", {}).get("gripper")

        # A jaw with no force sensing reports a grasp by FAILING to reach its commanded
        # position while carrying load.  Surface that as an explicit signal rather than
        # making every caller re-derive it from two numbers.
        measured = res["measured_deg"]
        stalled = (measured is not None and res.get("status") != "reached"
                   and (deg > measured if pct < 50.0 else deg < measured))
        res["possible_grasp"] = bool(stalled)
        if stalled:
            res["grasp_evidence"] = (
                f"jaw stopped at {measured:.1f} deg while commanded to {deg:.1f} deg; "
                "something is between the fingers")
        return res

    # ------------------------------------------------------------------ tools
    def go_home(self, **kw) -> dict:
        """Return to the recorded home pose.

        Home is a TICK pose recorded from this arm by its operator (`home_ticks` in the
        calibration file), not the middle of the URDF design limits. The design middle is a
        configuration nobody ever confirmed this particular arm can sit in; the recorded home is
        one an operator actually put it in and asked to be able to return to.

        Falling back to the design middle when no sweep has ever run is deliberate: a fresh
        profile should still be able to bring the arm to rest somewhere sensible rather than
        refusing to home at all.
        """
        home = self.calib.get("home_ticks")
        if home:
            return self.goto_joints(ticks_to_deg(home, self.calib), **kw)
        mid = {}
        for lname, urdf_name in LEROBOT_TO_URDF.items():
            lo, hi = self.kin.limits_deg[urdf_name]
            mid[lname] = (lo + hi) / 2.0
        return self.goto_joints(mid, **kw)

    def log(self, limit: int = 50) -> list[dict]:
        return self._log[-limit:]


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="ArmDriver read-only smoke test")
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--calibration", default="calib/out/so101_calibration.json")
    args = ap.parse_args()

    drv = ArmDriver(args.port, args.calibration, allow_motion=False)
    with drv:
        print(json.dumps(drv.state(), indent=2))
        print("\nhome pose would be:")
        mid = {k: (sum(drv.kin.limits_deg[LEROBOT_TO_URDF[k]]) / 2) for k in LEROBOT_TO_URDF}
        print("  ", {k: round(v, 1) for k, v in mid.items()})

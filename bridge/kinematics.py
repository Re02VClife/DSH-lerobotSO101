"""Forward/inverse kinematics for the SO-101 follower, straight from the official URDF.

No PyBullet, no placo, no pinocchio: the arm has a single serial chain of revolute
joints, so a hand-written FK plus a damped-least-squares IK is both sufficient and
much easier to reason about (and to instrument) than a physics engine.

Frame convention used everywhere in DS101
-----------------------------------------
    base frame = the arm's own base, origin at the table surface under the arm base,
                 +x pointing away from the arm toward the workspace/camera,
                 +y to the left, +z up.
    joint angles are in DEGREES, matching the URDF convention (0-360, mid-range
    poses are around 180 deg).

The URDF chain in docs/so101.urdf is:
    world -> base -> baseframe_frame -> Rotation -> Pitch -> Elbow -> Wrist_Pitch
          -> Wrist_Roll -> gripperframe_frame -> Jaw
`baseframe_frame` is the frame the arm's own base sits in; FK below returns the tool
point expressed relative to it.

IMPORTANT: joint angles here are *true* joint angles, not raw servo ticks.  Converting
ticks to degrees requires the per-arm homing offsets from calibration; feeding raw
tick degrees into this module silently produces garbage poses.  Use
`from_ticks()` only after calibration has populated the offsets.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

URDF_PATH = Path(__file__).resolve().parent.parent / "docs" / "so101.urdf"

#: revolute joints of the arm, in chain order (servo IDs 1..5)
ARM_JOINTS = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"]
#: + the gripper, which does not affect the wrist pose
ALL_JOINTS = ARM_JOINTS + ["Jaw"]

#: lerobot joint names <-> URDF joint names
LEROBOT_TO_URDF = {
    "shoulder_pan": "Rotation",
    "shoulder_lift": "Pitch",
    "elbow_flex": "Elbow",
    "wrist_flex": "Wrist_Pitch",
    "wrist_roll": "Wrist_Roll",
    "gripper": "Jaw",
}
URDF_TO_LEROBOT = {v: k for k, v in LEROBOT_TO_URDF.items()}


def _rpy_to_R(rpy: tuple[float, float, float]) -> np.ndarray:
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _axis_rotation(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / max(np.linalg.norm(axis), 1e-12)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle_rad) * K + (1 - math.cos(angle_rad)) * (K @ K)


def _T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


@dataclass
class JointSpec:
    name: str
    axis: np.ndarray
    origin_xyz: np.ndarray
    origin_rpy: tuple[float, float, float]
    lower_deg: float | None = None
    upper_deg: float | None = None


class SO101Kinematics:
    """FK + damped-least-squares IK for the SO-101 arm."""

    #: The URDF's `baseframe_frame` is NOT centred on the arm: the shoulder-pan
    #: (Rotation) joint axis sits at (-287.2, -336.1, +127.4) mm from it.  Working
    #: in baseframe_frame therefore produces coordinates that are offset by ~44 cm
    #: diagonally, which silently corrupts every calibration and every grasp target.
    #: Everything below is expressed relative to the shoulder-pan axis at table
    #: level, computed once from the URDF (see docs/frame_probe.py).
    BASE_OFFSET_M = np.array([-0.28724, -0.33610, 0.12740])

    def __init__(self, urdf_path: str | Path = URDF_PATH,
                 tool_offset_m: tuple[float, float, float] = (0.0, 0.0, -0.0981),
                 base_offset_m: tuple[float, float, float] | None = None) -> None:
        """tool_offset_m is the tool point expressed in `gripperframe_frame`.

        The default is the gripper tip offset taken from the URDF's own
        gripperframe_frame -> jaw geometry.  Measure yours: the physical gripper tip
        depends on how the jaws are assembled, and a 5 mm error here becomes a 5 mm
        error in every grasp.  Touch-probing the tip against a known point is the way
        to fix it (see docs/PLAN-...md stage 1, step 9).

        base_offset_m recentres the arm frame onto its own shoulder-pan axis.  Pass
        an overriding value only if the URDF changes.
        """
        self.urdf_path = Path(urdf_path)
        self.tool_offset_m = np.asarray(tool_offset_m, dtype=float)
        self.base_offset_m = (np.asarray(base_offset_m, dtype=float)
                              if base_offset_m is not None else self.BASE_OFFSET_M.copy())
        root = ET.parse(self.urdf_path).getroot()
        self._joints: dict[str, JointSpec] = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            a = j.find("axis")
            lim = j.find("limit")
            spec = JointSpec(
                name=j.get("name"),
                axis=np.array([float(v) for v in (a.get("xyz") if a is not None else "0 0 1").split()]),
                origin_xyz=np.array([float(v) for v in o.get("xyz").split()]),
                origin_rpy=tuple(float(v) for v in o.get("rpy").split()),
                lower_deg=math.degrees(float(lim.get("lower"))) if lim is not None else None,
                upper_deg=math.degrees(float(lim.get("upper"))) if lim is not None else None,
            )
            self._joints[spec.name] = spec

        self.chain = ["baseframe_frame", "Rotation", "Pitch", "Elbow", "Wrist_Pitch",
                      "Wrist_Roll", "gripperframe_frame"]

        # joint limits in degrees, in chain order, from the URDF
        self.limits_deg = {
            name: (self._joints[name].lower_deg, self._joints[name].upper_deg)
            for name in ALL_JOINTS
        }
        self.n_dof = len(ARM_JOINTS)

    # ------------------------------------------------------------------ forward
    def _fk_frames(self, q_deg: dict[str, float]) -> tuple[np.ndarray, dict[str, np.ndarray]]:
        M = np.eye(4)
        frames: dict[str, np.ndarray] = {}
        for name in self.chain:
            spec = self._joints[name]
            M = M @ _T(_rpy_to_R(spec.origin_rpy), spec.origin_xyz)
            if name in q_deg:
                M = M @ _T(_axis_rotation(spec.axis, math.radians(q_deg[name])), np.zeros(3))
            frames[name] = M.copy()
        return M, frames

    def fk(self, q_deg: dict[str, float], frame: str = "tool") -> np.ndarray:
        """Return the 4x4 pose in the DS101 arm frame (origin = shoulder-pan axis).

        frame: "tool" (gripper tip), "wrist" (gripperframe_frame), or any URDF link
        name in the chain.
        """
        q = {k: float(v) for k, v in q_deg.items()}
        M, frames = self._fk_frames(q)
        if frame == "tool":
            M = M @ _T(np.eye(3), self.tool_offset_m)
        elif frame in frames:
            M = frames[frame]
        else:
            raise KeyError(f"unknown frame {frame!r}")
        out = M.copy()
        out[:3, 3] = M[:3, 3] - self.base_offset_m
        return out

    def tool_position(self, q_deg: dict[str, float]) -> np.ndarray:
        return self.fk(q_deg)[:3, 3]

    # ------------------------------------------------------------------ jacobian
    def _jacobian(self, q_deg: dict[str, float], eps_deg: float = 0.05) -> np.ndarray:
        """Numerical 3xN position Jacobian with respect to the 5 arm joints."""
        base = self.tool_position(q_deg)
        J = np.zeros((3, self.n_dof))
        for k, name in enumerate(ARM_JOINTS):
            q2 = dict(q_deg)
            q2[name] = q2.get(name, 180.0) + eps_deg
            J[:, k] = (self.tool_position(q2) - base) / math.radians(eps_deg)
        return J

    # ------------------------------------------------------------------ inverse
    def ik(
        self,
        target_m: np.ndarray,
        q_seed: dict[str, float],
        *,
        max_iters: int = 200,
        tol_m: float = 0.0015,
        step_cap_deg: float = 4.0,
        damping: float = 0.05,
        table_clearance_m: float = 0.005,
        seed_aware: bool = True,
    ) -> dict:
        """Damped-least-squares IK for a Cartesian target, respecting joint limits.

        The arm has 5 positioning DOF, so position-only IK with a seed is
        well-posed for a top-down grasp: keep the wrist near vertical and let the
        solver find the shoulder/elbow branch nearest the seed.

        Returns dict with keys: ok, q_deg (lerobot names), iters, residual_m,
        reason, clamped_joints.
        """
        target = np.asarray(target_m, dtype=float).reshape(3)
        q = {name: float(q_seed.get(name, 180.0)) for name in ARM_JOINTS}
        lo = np.array([self.limits_deg[n][0] for n in ARM_JOINTS])
        hi = np.array([self.limits_deg[n][1] for n in ARM_JOINTS])
        # keep a margin off the hard limits: the servo will hit the mechanical stop
        # before the URDF limit if backlash is unfavourable
        margin = 4.0
        lo, hi = lo + margin, hi - margin

        reason = "max_iters"
        residual = float("inf")
        for it in range(max_iters):
            p = self.tool_position(q)
            err = target - p
            residual = float(np.linalg.norm(err))
            if residual < tol_m:
                reason = "converged"
                break
            J = self._jacobian(q)
            # damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 err
            A = J @ J.T + (damping ** 2) * np.eye(3)
            dq = J.T @ np.linalg.solve(A, err)
            dq_deg = np.degrees(dq)
            # cap the step so a bad iteration cannot throw the arm across the table
            cap = np.max(np.abs(dq_deg))
            if cap > step_cap_deg:
                dq_deg *= step_cap_deg / cap
            q_arr = np.array([q[n] for n in ARM_JOINTS]) + dq_deg
            q_arr = np.clip(q_arr, lo, hi)
            q = {n: float(v) for n, v in zip(ARM_JOINTS, q_arr)}

        p = self.tool_position(q)
        residual = float(np.linalg.norm(target - p))
        ok = residual < max(tol_m * 5, 0.008)
        if not ok and reason == "max_iters":
            reason = "unreachable_or_limited"

        return {
            "ok": ok,
            "q_deg": {URDF_TO_LEROBOT[n]: round(v, 3) for n, v in q.items()},
            "iters": it,
            "residual_mm": round(residual * 1000, 2),
            "reason": reason,
            "target_mm": [round(float(v) * 1000, 1) for v in target],
        }

    def reachable(self, target_m: np.ndarray, q_seed: dict[str, float], **kw) -> tuple[bool, dict]:
        res = self.ik(target_m, q_seed, **kw)
        return res["ok"], res

    # ------------------------------------------------------------------- helpers
    def ticks_to_deg(self, ticks: dict[str, int], homing_offset: dict[str, int],
                     direction_sign: dict[str, int] | None = None) -> dict[str, float]:
        """Raw servo ticks -> true joint degrees, using the per-arm calibration.

        lerobot's convention: a servo's present position counts 0..4095 over one
        revolution; the homing offset captured during calibration defines which tick
        corresponds to the joint's zero.  Without a calibration file this function
        cannot produce meaningful numbers.
        """
        sign = direction_sign or {k: 1 for k in LEROBOT_TO_URDF}
        out: dict[str, float] = {}
        for lname, urdf_name in LEROBOT_TO_URDF.items():
            if lname not in ticks or lname not in homing_offset:
                continue
            delta = (int(ticks[lname]) - int(homing_offset[lname])) % 4096
            out[urdf_name] = (delta * 360.0 / 4096.0) * sign.get(lname, 1)
        return out


def load_calibration(path: str | Path) -> dict:
    """Load a per-arm calibration file (homing offsets, limits, tool offset)."""
    import json

    return json.loads(Path(path).read_text(encoding="utf-8"))


if __name__ == "__main__":
    kin = SO101Kinematics()
    print("chain:", " -> ".join(kin.chain))
    print("\njoint limits (degrees, from the official URDF):")
    for name in ALL_JOINTS:
        lo, hi = kin.limits_deg[name]
        print(f"  {URDF_TO_LEROBOT[name]:<15} {lo:7.1f} .. {hi:7.1f}   (span {hi - lo:6.1f})")

    mid = {n: (kin.limits_deg[n][0] + kin.limits_deg[n][1]) / 2 for n in ALL_JOINTS}
    print(f"\nFK at mid-range pose: {np.round(kin.fk(mid)[:3, 3] * 1000, 1)} mm "
          f"(tool point, relative to the arm's own base frame)")

    span = kin.limits_deg
    print("\nreach sanity check (arm frame: origin at the shoulder-pan axis, z=0 at the "
          "shoulder-pan joint centre)")
    pts = []
    for rot in np.linspace(span["Rotation"][0] + 6, span["Rotation"][1] - 6, 25):
        for pit in np.linspace(span["Pitch"][0] + 6, span["Pitch"][1] - 6, 15):
            for elb in np.linspace(span["Elbow"][0] + 6, span["Elbow"][1] - 6, 15):
                q = dict(mid)
                q.update({"Rotation": rot, "Pitch": pit, "Elbow": elb})
                pts.append(kin.tool_position(q))
    pts = np.array(pts)
    r_xy = np.linalg.norm(pts[:, :2], axis=1)
    print(f"  horizontal reach radius: {r_xy.min() * 1000:.0f} .. {r_xy.max() * 1000:.0f} mm")
    print(f"  tool height z:           {pts[:, 2].min() * 1000:.0f} .. {pts[:, 2].max() * 1000:.0f} mm")
    print(f"  table plane (z = -{kin.base_offset_m[2] * 1000:.1f} mm) "
          f"means a grasp point on the table is at z = {-kin.base_offset_m[2] * 1000:.1f} mm")

    print("\nIK round-trip over a grid of table-level targets:")
    n_ok = 0
    worst = 0.0
    for x in (0.15, 0.20, 0.25, 0.30):
        for y in (-0.15, 0.0, 0.15):
            for z in (-0.05, 0.0, 0.08):
                tgt = np.array([x, y, z])
                res = kin.ik(tgt, mid)
                if res["ok"]:
                    n_ok += 1
                    worst = max(worst, res["residual_mm"])
    total = 4 * 3 * 3
    print(f"  solvable: {n_ok}/{total} targets, worst residual {worst:.2f} mm")
    res = kin.ik(np.array([0.20, 0.0, 0.08]), mid)
    print(f"  example target (200, 0, 80) mm -> ok={res['ok']} residual={res['residual_mm']} mm "
          f"iters={res['iters']}")
    print(f"  q = {res['q_deg']}")

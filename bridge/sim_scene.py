"""A small closed-loop bench simulator: blocks that move when the arm moves them.

`bridge/sim.py` simulates the servos and `bridge/fake_camera.py` the photons. Neither of
them knows anything about the world, so a pick-and-place could not be *verified* — the
scene looked identical before and after. This module closes that gap: it tracks where each
object is, decides whether a gripper close actually caught anything, and follows the
carried object as the arm moves, so the perception step after a placement reports the
object at its new position.

That is what makes "看图→决策→抓取→放置→验证" testable without hardware, and the
verification step is the part that matters: a pick-and-place that reports success without
re-observing the world is reporting its own intentions, not the outcome.

Deliberate simplifications, stated so they are not mistaken for physics:

  * an object is "caught" if the tool point is within its footprint and the closing
    direction points away from the object face the jaws approach from — no friction, no
    deformation, no slip;
  * a carried object is placed when the jaws open, and lands at the tool tip's projected
    position at that moment;
  * the object keeps its orientation, rotating about its own centre rather than about the
    tool tip as a real grasped block would.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from bridge.servo_bus import ADDR_PRESENT_POSITION, ADDR_TORQUE_ENABLE, JOINT_IDS
from bridge.sim import SimServoBus, sim_calibration
from bridge.kinematics import LEROBOT_TO_URDF

#: the tool point the forward kinematics reports sits this far above the jaws that actually
#: touch the object (see calib.perception.tool_target_for_grasp).  Without it a grasp pose
#: looks like a wrist floating 98 mm above the block and every height test is meaningless.
JAW_OFFSET_MM = 98.0

#: the jaws must be within this of the object's top face to count as closing on it (mm)
JAW_GRASP_BAND_MM = 30.0

#: how far the jaws must back off the width they stalled at before the object is let go,
#: in ticks.  A 40 mm block stalls the jaws at 2150 ticks and fully open is 1537, so this
#: is a quarter of the way open -- far outside anything the plant's own integration noise
#: could produce.
RELEASE_MARGIN_TICKS = 150

#: a stall counts as a grasp only within this of the object's width (ticks)
STALL_TOLERANCE_TICKS = 150


@dataclass
class SimObject:
    """A block on the table, tracked in world coordinates."""
    name: str
    centre_mm: np.ndarray          # (x, y) in the table plane
    size_mm: tuple[float, float]
    height_mm: float
    bgr: tuple[int, int, int]
    yaw_deg: float = 0.0
    #: set while the jaws hold it; cleared when released
    carried: bool = False
    #: the width the jaws stall at when closing on this object, in ticks from the midpoint
    grip_width_ticks: int = 2150

    def corners_mm(self) -> np.ndarray:
        """The footprint corners, (x, y) in the table plane, rotated by yaw."""
        w, d = self.size_mm
        local = np.array([[-w / 2, -d / 2], [w / 2, -d / 2],
                          [w / 2, d / 2], [-w / 2, d / 2]])
        a = math.radians(self.yaw_deg)
        R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        return local @ R.T + self.centre_mm

    def top_plane_z_mm(self, table_z_mm: float) -> float:
        return table_z_mm + self.height_mm


class SimScene:
    """A `SimServoBus` coupled to the blocks on the table.

    Attach it with `bus.plant = scene` and the bus calls `step()` on every integration tick.
    That placement is the entire point of the class. The first version required a caller to
    invoke `sync()` by hand, so the jaw's travel limit only applied while somebody happened
    to be looking; the motion layer closes the jaws over several rate-limited commands
    inside a single tool call, so the clamp arrived too late, the jaw sailed past the block,
    and the driver reported a clean "reached" with `possible_grasp: false` for a grasp that
    had in fact succeeded. Physics belongs in the integration loop, not in a caller's
    diligence.
    """

    def __init__(
        self,
        bus: SimServoBus,
        kin,
        *,
        table_z_mm: float = -115.0,
        objects: list[SimObject] | None = None,
    ) -> None:
        self.bus = bus
        self.kin = kin
        self.calib = sim_calibration(kin)
        self.table_z_mm = table_z_mm
        self.objects: list[SimObject] = objects if objects is not None else default_objects()
        self.held: SimObject | None = None
        self.events: list[dict] = []
        self._grasp_at: np.ndarray | None = None
        #: the jaw's tick value at the last moment no object was under the jaws.  This is
        #: the evidence for "were the fingers open when they came down on this block?" and
        #: it has to be sampled while nothing is underneath, because once the travel limit
        #: engages the jaw's reported position is the limit, not where it actually came from.
        self._open_ticks_at_arrival: int | None = None
        bus.plant = self                      # the bus drives the world from now on

    # ------------------------------------------------------------------ helpers
    def tool_tip_mm(self) -> np.ndarray:
        """The tool point in arm-frame millimetres, from the bus's own tick readback."""
        raw = {n: st.ticks for n, st in self.bus.read_all().items()}
        calib = self.calib
        deg = {}
        for name, mid in calib["middle_ticks"].items():
            sign = calib["direction_sign"].get(name, 1)
            deg[name] = calib["middle_pose_deg"][name] + (raw[name] - mid) * 360.0 / 4096 * sign
        q = {LEROBOT_TO_URDF[k]: v for k, v in deg.items() if k in LEROBOT_TO_URDF}
        T = self.kin.fk(q, frame="tool")
        return T[:3, 3] * 1000.0

    def object_positions(self) -> list[SimObject]:
        return [o for o in self.objects]

    def block_dicts(self) -> list[dict]:
        """The shape `bridge.fake_camera.SyntheticOverheadCamera.render` consumes."""
        return [
            {
                "name": o.name,
                "centre_mm": (float(o.centre_mm[0]), float(o.centre_mm[1])),
                "size_mm": o.size_mm,
                "height_mm": o.height_mm,
                "bgr": o.bgr,
            }
            for o in self.objects
            if not o.carried
        ]

    # --------------------------------------------------------------- the physics
    def step(self) -> None:
        """One physics tick, driven by `SimServoBus` on every integration step.

        The grasp rule is a CONTINUOUS constraint rather than an edge trigger, and that is
        what makes it work. The motion layer closes the jaws over several rate-limited
        commands, so the tick value jumps from well below a block's width to fully shut
        between two observations; an edge-triggered "did we just cross the width?" test
        misses the block entirely. Instead, whenever a graspable block sits under the tool
        point the jaw's travel is capped at that block's width for as long as the tool is
        there -- exactly what a real block does -- and the resulting stall is what tells the
        driver that something is between the fingers.

        Idempotent. Calling it directly as well is harmless.
        """
        ticks = self.bus._read2(JOINT_IDS["gripper"], ADDR_PRESENT_POSITION)
        torque = self.bus._read1(JOINT_IDS["gripper"], ADDR_TORQUE_ENABLE)

        if self.held is not None:
            tip = self.tool_tip_mm()
            self.held.centre_mm = np.array([tip[0], tip[1]])
            self.held.carried = True
            # Released only when the jaws have opened AND the arm has carried the block
            # somewhere. Releasing on the jaws alone dropped the block the instant they
            # cracked open, at the source, before the arm had gone anywhere -- and the jaws
            # were still near the block's width, so the next close "caught" it again. Moving
            # first is also what a person does.
            #
            # "Open" is measured against the width the jaws actually stalled at, never
            # against an absolute tick value: a thin block stalls the jaws long before they
            # are shut, so an absolute threshold calls that grasp "already open" and drops
            # the block on the first lift.
            open_enough = ticks < self.held.grip_width_ticks - RELEASE_MARGIN_TICKS
            carried_away = (
                self._grasp_at is not None
                and float(np.linalg.norm(np.array([tip[0], tip[1]]) - self._grasp_at)) > 60.0
            )
            if open_enough and carried_away:
                self._release(tip)
            return

        candidate = self._candidate_under_tool()
        if candidate is None or not torque:
            self._open_ticks_at_arrival = ticks
            self.bus.cfg.grasped_width_ticks = None
            return

        # Jaws that were already shut when they arrived at the object cannot be around it.
        # Without this, commanding the jaws closed in mid-air and then lowering them onto a
        # block reads as a grasp of a block the fingers never enclosed.
        if (self._open_ticks_at_arrival is not None
                and self._open_ticks_at_arrival >= candidate.grip_width_ticks):
            self.bus.cfg.grasped_width_ticks = None
            return

        self.bus.cfg.grasped_width_ticks = {"gripper": candidate.grip_width_ticks}
        self.bus._apply_gripper_limit()
        ticks = self.bus._read2(JOINT_IDS["gripper"], ADDR_PRESENT_POSITION)

        # Committed once the jaws have come to rest at the block's width. The limit pins
        # the jaw exactly there, so the tolerance only has to absorb the plant's integer
        # tick arithmetic; a jaw that closed on nothing sails past it to its shut limit and
        # a jaw still on its way in has not arrived yet -- neither registers a grasp.
        committed = (candidate.grip_width_ticks
                     <= ticks < candidate.grip_width_ticks + STALL_TOLERANCE_TICKS)
        if committed:
            candidate.carried = True
            self.held = candidate
            self._grasp_at = np.array([self.tool_tip_mm()[0], self.tool_tip_mm()[1]])
            self.bus.place_object_in_gripper(candidate.grip_width_ticks)
            self.events.append({"event": "grasp", "object": candidate.name,
                                "tool_mm": [round(float(v), 1) for v in self.tool_tip_mm()]})

    def _candidate_under_tool(self) -> SimObject | None:
        """The object the jaws are in a position to close on, if any.

        Two conditions, and both are necessary. The tool point must be inside the object's
        footprint, so the jaws are above it; and the JAWS -- not the tool point, which sits
        `JAW_OFFSET_MM` higher up the wrist -- must be down at the object's top face. The
        second is not decoration: without it, commanding the jaws shut while the arm is
        still 60 mm above the block stalls them at the block's width, and the driver
        faithfully reports a grasp of a block the fingers never reached.
        """
        tip = self.tool_tip_mm()
        jaw_z = tip[2] - JAW_OFFSET_MM
        for o in self.objects:
            if o.carried:
                continue
            top_z = o.top_plane_z_mm(self.table_z_mm)
            if abs(jaw_z - top_z) > JAW_GRASP_BAND_MM:
                continue
            if self._footprint_contains(o, tip):
                return o
        return None

    def _release(self, tip: np.ndarray) -> None:
        o = self.held
        self.held = None
        self.bus.cfg.grasped_width_ticks = None
        o.carried = False
        o.centre_mm = np.array([tip[0], tip[1]])
        self.events.append({"event": "place", "object": o.name,
                            "at_mm": [round(float(v), 1) for v in tip]})

    def _footprint_contains(self, o: SimObject, point_mm: np.ndarray) -> bool:
        """Point-in-rotated-rectangle, in the table plane."""
        w, d = o.size_mm
        rel = np.array([point_mm[0], point_mm[1]]) - o.centre_mm
        a = math.radians(-o.yaw_deg)
        R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        local = R @ rel
        return abs(local[0]) <= w / 2 and abs(local[1]) <= d / 2

    def held_object(self) -> SimObject | None:
        return self.held


def default_objects() -> list[SimObject]:
    """Three blocks inside the visible table band, one of them tall enough to be picked."""
    return [
        SimObject("red_block", np.array([200.0, -60.0]), (40.0, 40.0), 45.0, (40, 40, 200)),
        SimObject("blue_block", np.array([300.0, 120.0]), (45.0, 45.0), 60.0, (200, 60, 40)),
        SimObject("green_pad", np.array([140.0, 60.0]), (70.0, 70.0), 12.0, (60, 190, 60)),
    ]


def make_scene(table_z_mm: float = -115.0, **kw) -> tuple[SimServoBus, SimScene, object]:
    """Convenience builder for tests: (bus, scene, kinematics)."""
    from bridge.kinematics import SO101Kinematics

    kin = SO101Kinematics()
    bus = SimServoBus(allow_motion=True)
    scene = SimScene(bus, kin, table_z_mm=table_z_mm, **kw)
    return bus, scene, kin

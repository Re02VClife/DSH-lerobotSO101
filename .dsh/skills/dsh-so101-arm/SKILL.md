---
name: dsh-so101-arm
description: Operate the DS101 SO-101 robot arm bench through the DS101 bridge and its arm_* tools. Use when asked to move the arm, pick or place an object, calibrate the bench, interpret a failed grasp or a refused motion, or work out why a Cartesian target is unreachable. Covers the mount geometry, the safety envelope, and the failure modes that look like code bugs but are not.
---

# Operating the DS101 SO-101 bench

This bench is a 6-DOF SO-101 follower driven by a LeRobot-style Feetech bus on COM10, with
a fixed overhead RealSense D435 and a wrist camera. The agent drives it through the DS101
bridge's `arm_*` tools. Read this before moving anything.

## Before you touch the arm

**Check whether the arm is actually calibrated.** Everything below assumes
`calib/out/so101_calibration.json` came from a real joint-zero calibration. If it did not,
joint angles, FK and every Cartesian target are silently wrong, and the arm will still
happily move.

```powershell
.\.venv\Scripts\python.exe -c "import json;d=json.load(open('calib/out/so101_calibration.json'));print('created:',d['created']);print('homing_written:',d.get('homing_written'))"
```

`created: simulation` or `homing_written: false` means **do not trust any pose number**.
Ask the operator to run `calib/arm_zero.py`; it needs a human to hold the arm at the
middle of every joint's range.

**Start the bridge in the right mode.** The bridge owns COM10 and the RealSense, so
exactly one instance may run:

- `bridge/server.py` — real hardware, **read-only** unless `--allow-motion` is passed;
- `bridge/server.py --allow-motion` — real hardware, motion permitted;
- `bridge/server.py --simulate` — synthetic servos *and* synthetic camera, no hardware at
  all. Use this to rehearse a sequence before doing it for real.

Stop it with `curl -X POST http://127.0.0.1:8731/shutdown`. **Never hard-kill it**: a
killed bridge can leave the COM port handle stuck in the kernel, and after that nothing
user-space can open COM10 until the USB adapter is physically unplugged.

## The geometry you must keep in mind

The arm frame has its origin at the shoulder-pan axis, **not** at the table and **not** at
the URDF's `baseframe_frame` (that one is offset by about (-287, -336, +127) mm and using
it puts every coordinate ~44 cm out). +x points away from the arm toward the workspace,
+y left, +z up.

| Quantity | Value on this bench |
|---|---|
| Table surface | z = **-115 mm** (shoulder axis is 115 mm above it) |
| Camera position | 440 mm forward, 625 mm above the table |
| Camera pitch | **16 deg off vertical**, leaning back toward the arm |
| Reachable radius at table height | about **50-430 mm** |
| Overhead camera sees | roughly a **720 mm deep strip**, x from -150 to +570 mm |
| Safe first moves | keep the tip at z >= -60 mm and radius <= 300 mm until validated |

Two consequences that are easy to forget:

- **Do not aim at the geometric maximum reach.** The wrist target, not the tool tip, is
  what IK solves for, and the jaw offset (~98 mm) means a tip target that looks reachable
  can put the wrist outside the envelope.
- The overhead camera's view is a **tilted band**, not a rectangle around the base. An
  object 700 mm out is in arm range but outside the frame; an object at the near edge of
  the frame may be inside the arm's own footprint.

## The operating loop

1. `arm_status` — confirm the bridge answers, read voltage/temperature, and check that
   motion is permitted. Never skip this: a read-only bridge will refuse every move, and
   that is a configuration fact, not a fault.
2. `arm_scene` — structured perception. You get numbers, not an image: per object, the
   `centroid_base_mm`, `extent_mm`, `height_above_table_mm` and median `colour_bgr`. The
   table plane and its inlier fraction come with it.
3. Pick a target **by table position, not by array index** — the ordering changes between
   frames as objects move.
4. `arm_plan` — dry run. It solves IK and reports residual and reachability **without
   moving**. Always do this before `arm_move_to`.
5. `arm_move_to` — approach height first (grasp height + ~60 mm), then descend.
6. `arm_gripper` at 0 % to close, then read `possible_grasp` from the reply.
7. Lift, move, release, then re-run `arm_scene` to **verify** the object actually left.

## Signals that mean something specific

| You see | It means |
|---|---|
| `status: "reached"` | the joints got within 1.5 deg of the target — the move is done |
| `status: "clamped"` with `clipped: [...]` | the target was outside the joint envelope; the arm moved as far as allowed and **did not arrive** |
| `status: "budget_exhausted"` | the rate limiter (15 deg per command) needs another call; nothing is wrong, just repeat |
| `commands_issued` > 1 | normal for any large move; this is the limiter working |
| `possible_grasp: true` with `grasp_evidence` | the jaw stalled short of its commanded position: **something is between the fingers** |
| `possible_grasp: false` on a close | the jaw reached full closed travel: **the grasp is empty** — re-plan, do not lift and hope |
| `ik_failed:unreachable_or_limited` | the target is outside the reachable set; do not retry the same point |
| `no extrinsics calibration yet` | `T_base_cam` is missing, so pixel+depth cannot become an arm-frame point; only `arm_scene`'s table-relative numbers are usable |
| `bridge unreachable` | the service is not running; start it as above |

**`possible_grasp` is the only grasp feedback there is.** These servos have no force
sensing. A jaw that closes on nothing travels its whole range; a jaw that closes on an
object stalls early. Never assume a grasp succeeded from the fact that the move
succeeded — they are independent facts.

## Failure modes that look like code bugs but are not

**A flat calibration board is unusable on the overhead camera.** At 16 deg off vertical the
camera sees a board lying on the table at roughly **74 degrees incidence**, compressed
about 3.6x in one image direction. Corner detection rejects that, and a real camera would
too. Tilt the board 45-60 degrees toward the camera before capturing — run
`calib/bench_geometry.py` to see which placements actually fit the frame. The board is
ChArUco rather than a checkerboard specifically because the arm occludes part of a fixed
overhead view, and a plain checkerboard returns nothing once a corner region is covered.

**Intrinsics with no depth variation produce a wildly wrong focal length.** If every
capture sits at the same distance, focal length and range trade off against each other and
`cv2.calibrateCamera` returns a focal length that can be off by ~90% **with a sub-pixel RMS
that looks excellent**. Vary the distance across the set. This is not a detector problem.

**A perceived object centre is not the object's footprint centre.** At this viewing angle
the top face projects toward the camera, so the top-face centroid is biased by roughly
`height * tan(incidence)` — measured at ~15 mm for a 45 mm block, which is enough to make
the jaws close behind the object's middle. `footprint_centre_base_mm()` back-projects that
bias onto the table; use it rather than the raw centroid when the true centre matters.

**A table plane with a low inlier fraction means clutter, not a broken fit.** A clean
synthetic table gives ~0.99. A real bench reading around 0.5 usually has a cluttered or
non-planar workspace; ask for it to be cleared before trusting object heights.

**`arm_scene`'s height is trustworthy; its plan-position is biased.** Together with the
point above: heights come out accurate to a few mm, but the reported centre drifts toward
the camera with object height. Do not "fix" that by nudging the camera calibration.

**OpenCV 5 removed several functions the usual tutorials use.** `cv2.calibrateHandEye`,
`cv2.aruco.detectMarkers`, `estimatePoseCharucoBoard` and `undistortImage` are **absent**.
The working route is `CharucoDetector.detectBoard` -> `Board.matchImagePoints` ->
`cv2.solvePnP`, which is what this repo uses. Do not reach for a `calibrateHandEye` recipe.

## What not to do

- Do not raise a safety threshold to make a refused move succeed. Voltage (4.2 V),
  temperature (65 C), step (15 deg), and the joint envelope exist because the arm can hurt
  itself or the bench. A refusal is information.
- Do not run a Cartesian sequence without a prior `arm_plan`.
- Do not assume `ok: true` means "arrived" without reading `status`.
- Do not move the arm while anyone's hands are in the workspace, and treat the software
  `arm_estop` as a convenience, **not** as the emergency stop — that is the physical switch
  on the servo power, and it must exist before any autonomous motion.
- Do not re-run a calibration to fix a symptom. If poses disagree with a tape measure,
  suspect the joint-zero calibration's `direction_sign` before anything else.

## Verifying the stack without hardware

Both of these run in seconds and need no arm:

```powershell
.\.venv\Scripts\python.exe calib\selftest.py     # calibration chain, synthetic
.\.venv\Scripts\python.exe calib\loop_test.py    # closed loop: perceive, plan, move, grasp
```

`loop_test.py` drives the **production** `ArmDriver` against a simulated plant, so a pass
means the logic is right and the remaining risk is mechanical (repeatability, backlash,
whether the table is actually level).

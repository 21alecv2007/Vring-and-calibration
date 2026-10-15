# Cyton IK controller

Cartesian control of the Cyton Gamma 300 end effector: type or nudge a target
position (mm) and orientation (roll, pitch, yaw in degrees), solve inverse
kinematics, preview the solution, and move the arm through the existing
M5 -> Uno gateway. Requires `numpy` and `pyserial` (both in the repo `.venv`).

| File | Purpose |
|---|---|
| `cyton_kinematics.py` | Pure-numpy model: forward kinematics, geometric Jacobian, damped-least-squares IK with joint limits, null-space rest-pose bias and restarts, servo raw <-> radian calibration |
| `cyton_ik_gui.py` | Tkinter controller (3-D wireframe preview, target nudging, coordinated moves, continuous tracking, gripper, calibration panel) |
| `test_kinematics.py` | Jacobian vs finite differences, RPY / rotation-vector round trips, IK round trips in all three modes, calibration round trip |
| `cyton_calibration.json` | Created by "Save calibration": per-joint servo ID, zero offset, direction sign |

Start with `launch_ik_gui.sh` (macOS/Linux), `launch_ik_gui.command`
(double-click on macOS) or `launch_ik_gui.bat` (Windows). An optional argument
forces the serial port. Run the tests with `python test_kinematics.py`.

## Model

Geometry is the community URDF of the Cyton Gamma 300 (Tyler Slabinski's
model as distributed in the `cyton_gamma_300_description` ROS packages):
seven revolute joints, each a pure translation along the parent z axis
followed by a rotation about a fixed axis.

| # | Joint | Offset from previous joint (m) | Axis | Software limit | Servo |
|--:|---|---|---|---|---|
| 1 | shoulder_roll | 0.05315 | +z | ±151° | ID 8, MX-28 |
| 2 | shoulder_pitch | 0.063 | +x | ±113° | ID 9, MX-64 |
| 3 | elbow_roll | 0.089 | +z | ±151° | ID 10, MX-28 |
| 4 | elbow_pitch | 0.052 | +x | ±113° | ID 11, MX-28 |
| 5 | elbow_yaw | 0.072 | −y | ±113° | ID 12, MX-28 |
| 6 | wrist_pitch | 0.0718 | +x | ±113° | ID 13, MX-28 |
| 7 | wrist_roll | 0.051425 | +z | ±151° | ID 14, MX-28 |
| — | tool point | (−0.0023, 0.0079, 0.0794) from wrist_roll | — | — | gripper ID 15, AX-12A |

Software limits are the ranges the ROS `dynamixel_controllers` config for this
arm model drives (`cyton_gamma_300_controller.yaml`), wider than the URDF's
conservative ±105° pitch limits. Each servo's own stored CW/CCW window is
intersected with them, and the Uno gateway rejects any move outside it.

Base frame: at the bottom of the arm, z up. Joint zero = arm pointing straight
up; the tool point is then 532 mm above the base. Orientation targets use the
ROS convention `R = Rz(yaw) · Ry(pitch) · Rx(roll)`; the tool z axis is the
approach direction (out through the gripper).

## IK modes

- **Position + full orientation** (6-D): all seven joints constrained, one
  redundant DOF resolved toward a mildly bent rest pose.
- **Position + approach axis** (5-D): aligns the tool z axis only and leaves
  roll about it free. Much larger reachable set; good for pointing/grasping.
- **Position only** (3-D).

Solver: damped least squares on a weighted 6-vector error (position in metres,
rotation vector scaled by 0.3 m/rad), adaptive damping, per-iteration step
clamp, joint-limit clamping, a null-space pull toward the rest pose through
an undamped projector, and up to four restarts (rest pose, then random seeds)
when the warm start fails. Tolerances 0.5 mm / 0.5°. A solve takes a few
milliseconds.

## Calibration (do this once, before trusting any Cartesian move)

The model needs to know, for each joint, which servo it is, which raw reading
corresponds to the URDF zero angle, and whether raw counts increase with the
model's positive rotation. Defaults: IDs 8–14 in chain order, zero = 2048,
sign = +1. The ROS driver config for the same arm model uses exactly this
joint order (IDs 0–6 there; this arm was re-numbered to 8–15), no flipped
joints, and zero ticks between 2045 and 2070, so the defaults should be
within about 2° per joint. Refine them like this:

1. Connect, keep torque off, and by hand put the arm in the URDF zero pose:
   every link vertical, arm pointing straight up, gripper jaws opening along
   the base x axis. Press **Zero ALL here**.
2. Enable torque, then nudge one joint at a time (use the joint GUI's +10 or
   the manual `MOVE` line) and watch the wireframe. If the drawn arm bends the
   opposite way from the real arm, tick **flip** for that joint. If a
   different link moves, fix the servo ID spinbox.
3. Press **Save calibration**. It is written next to the scripts and loaded on
   start.

Until calibration is done the preview will not match the real arm and IK
moves will go somewhere unexpected. Test the first Cartesian moves with the
arm supported, the workspace clear, and a low max speed.

## Setting the target

- Type values (Enter re-solves) or use the +/− nudge buttons with the step
  sizes.
- **Grab it in the 3-D view.** The target carries a translate gizmo: drag
  the gold centre knob to slide the target in the ground plane, drag the
  red / green / blue knob to move along world X / Y / Z only, and use the
  scroll wheel for height. Dragging empty space still orbits the view.
- **Keyboard jog** (click the view first): arrows move X/Y, PgUp/PgDn move Z,
  Q/E yaw, A/D pitch, Z/C roll, all by the step sizes.
- **Target ← current** snaps the target to the arm's live pose.

## Moving

- **Go to target** sends one `MOVE` per joint with speeds scaled so that all
  joints finish together; the largest joint change runs at *max speed*.
- **Track target continuously** runs a ~8 Hz loop that sends the newest IK
  solution whenever it changed, from any source: gizmo drag, keys, nudges,
  typed values. It uses `GROUP` when the gripper is torqued too, otherwise
  coordinated `MOVE`s. Turning it on with torque off offers to enable the
  arm; while any joint is un-torqued or the solver has not converged it
  pauses and says so under the checkbox, and a red TRACKING banner shows in
  the view while it is live. STOP, Escape, or unticking the box ends it.
- Escape or the big STOP button turns torque off on every servo, exactly as
  in the joint GUI. Closing the window also sends `STOP`.

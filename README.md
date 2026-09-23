# nero-jacket-fold

Scripted jacket folding demo for the AgileX Nero arm, following the same
pattern as `nero-carrot-pickplace`: 10 pre-taught joint-space waypoints
(starting and ending at `pose_90deg`) executed directly via
`FollowJointTrajectory`, with start/stop exposed as ROS 2 services.

## Build

```bash
cd ~/agx_arm_ws/src
git submodule add <this-repo-url> nero-jacket-fold
cd ~/agx_arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select agx_jacket_demo
source install/setup.bash
```

## Launch the arm

**Simulation:**
```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=agx_gripper
```

**Real hardware** (CAN setup per `nero-carrot-pickplace`'s README first):
```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can_piper arm_type:=nero effector_type:=agx_gripper \
  auto_control_gate:=true
```
Wait for `Agx_arm feedback is ready, control is now enabled` before continuing.

## Run

```bash
source ~/agx_arm_ws/install/setup.bash
ros2 run agx_jacket_demo jacket_sequence_node
```
Wait for `Ready.`

## Start / stop

```bash
ros2 service call /start_jacket_sequence std_srvs/srv/Trigger
```
```bash
ros2 service call /stop_jacket_sequence std_srvs/srv/Trigger
```

## Sequence

| Step | State | Description |
|---|---|---|
| 0 | `pose_90deg_start` | Return to 90° rest pose |
| 1 | `state1` | Turn to jacket top |
| 2 | `state2` | Move to jacket collar |
| 3 | `state3` | Turn to jacket collar |
| 4 | `state4` | Grip jacket |
| 5 | `state5` | Pick it up |
| 6 | `state6` | Fold it |
| 7 | `state7` | Move to let-go position |
| 8 | `state8` | Let go (release) |
| 9 | `pose_90deg_end` | Return to rest |

## Safety notes

Same as `nero-carrot-pickplace`: no collision checking, raw pre-taught
trajectories — test in simulation first every time, ensure the jacket is
positioned exactly as it was when these waypoints were captured, and keep
the workspace clear before every run.

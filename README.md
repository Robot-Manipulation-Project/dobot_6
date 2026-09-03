# Dobot 6 - Robot Manipulation Project

This repository contains Python and ROS 2 interfaces for controlling a **Dobot Magician** robotic arm over USB.

## Repository layout

- `6.1/` - standalone Python socket-based control stack (`dobot_server.py` + `dobot_client.py`)
- `6.2/mxen_ws/src/dobot_interface/` - ROS 2 custom interfaces (actions/services)
- `6.2/mxen_ws/src/dobot/` - ROS 2 Python package with drivers and nodes

## Features

- USB serial communication with Dobot Magician (`/dev/ttyACM0-7` autodetection)
- Homing trigger service
- Suction cup trigger service
- Joint state and Cartesian pose publishing
- Joint-space PTP action server
- Cartesian-space PTP action server (with IK reachability checks)
- Pick-and-place service workflow

## Prerequisites

- Ubuntu + ROS 2 (tested with a standard `colcon` workflow)
- Python 3
- `pyserial`
- A connected Dobot Magician arm (typically on `/dev/ttyACM*`)

## ROS 2 package setup (6.2)

From workspace root:

```bash
cd /home/runner/work/dobot_6/dobot_6/6.2/mxen_ws
colcon build
source install/setup.bash
```

## Launch all Dobot nodes

```bash
ros2 launch dobot dobot_launch.xml
```

The launch file starts:

- backend server script (`dobot_server.py`)
- `homing_node`
- `joint_state_node`
- `suction_cup_node`
- `joint_ptp_node`
- `cartesian_ptp_node`
- `pick_and_place_node`

## Interfaces

### Services

- `homing` (`std_srvs/Trigger`) - starts homing
- `suction_cup` (`std_srvs/Trigger`) - toggles suction cup ON/OFF
- `pick_and_place` (`dobot_interface/srv/PickAndPlace`)
  - request: `float32[4] pick_pose`, `float32[4] place_pose` (`[x, y, z, r]`)
  - response: `bool success`, `string message`

### Actions

- `set_joint_ptp` (`dobot_interface/action/JointPTP`)
  - goal: `float64[4] joint_goal`
  - feedback: `float64[4] joint_present`
  - result: `bool success`

- `set_cartesian_ptp` (`dobot_interface/action/PosePTP`)
  - goal: `float32[4] pose_goal`
  - feedback: `float32[4] pose_present`
  - result: `bool success`

### Topics

- `joint_state` (`sensor_msgs/JointState`)
- `pose` (`geometry_msgs/Pose`)

## Quick usage examples

```bash
# Trigger homing
ros2 service call /homing std_srvs/srv/Trigger {}

# Toggle suction cup
ros2 service call /suction_cup std_srvs/srv/Trigger {}

# Send joint goal [j1, j2, j3, j4]
ros2 action send_goal /set_joint_ptp dobot_interface/action/JointPTP "{joint_goal: [0.0, 20.0, 10.0, 0.0]}"

# Send Cartesian goal [x, y, z, r]
ros2 action send_goal /set_cartesian_ptp dobot_interface/action/PosePTP "{pose_goal: [200.0, 0.0, 20.0, 0.0]}"
```

## Legacy standalone API (6.1)

`6.1/` provides a pure-Python path without ROS 2:

1. Run `dobot_server.py` to expose a local socket API
2. Use `DobotClient` in `dobot_client.py` to send commands

The client supports:

- `start_homing()`
- `get_joint_state()`
- `is_goal_valid(j1, j2, j3, j4)`
- `set_joint_ptp(j1, j2, j3, j4)`
- `set_suction_cup(enable)`
- `stop_current_action()`

## Safety notes

- Keep a clear workspace around the robot before commanding motion.
- Validate target poses/goals before execution.
- Be prepared to stop motion immediately if behavior is unexpected.

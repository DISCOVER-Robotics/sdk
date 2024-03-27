# ROS Interface

The ROS adapter layer for [`arm-control`](https://git.qiuzhi.tech/airbot-play/control/arm-control)

## 1. Install

### 1.1 Install latest `arm-control`

Install the deb packages from [releases of `arm-control`](https://git.qiuzhi.tech/airbot-play/control/arm-control/-/releases)

### 1.2 (Optional) Install depencencies of JoyStick

Run with root privileges:

```shell
apt install ros-noetic-joy ros-noetic-tf ros-noetic-kdl-parser
```

### 1.3 Create ROS workspace and install ROS interface

```shell
source /opt/ros/noetic/setup.bash
mkdir -p ri_ws/src/
cd ri_ws/src/ && git clone https://git.qiuzhi.tech/airbot-play/control/ros-interface.git && cd ..
source devel/setup.bash

roslaunch ros_interface airbot_arm.launch [urdf:=</path/to/urdf/file>]
```

Now you can control the arm by ROS and JoyStick.

**Note**: The default urdf does not contain end effector. If you wish to perform precise end control, a custom urdf file with `link6` as the target end is necessary.

## 2. Usage

### 2.1 ROS Topic Reference

- `/airbot_play/arm_pose`: Publisher, `geometry_msgs/Pose`, publish end pose relative to base
- `/airbot_play/pose_cmd`: Publisher, `sensor_msgs/JointState`, publish joint states / velocities / efforts
- `/airbot_play/joint_states`: Subscriber, `geometry_msgs/Pose`, receive target end pose and start moving
- `/airbot_play/joint_cmd`: Subscriber, `sensor_msgs/JointState`, receive target joint position / velocities (not supported for now) / efforts (not supported for now) and start moving

### 2.2 JoyStick

Set the joystick to "DirectInput" mode (put the switch to "D" mode on Logitech Wireless Gamepad F710)

- Left stick horizontal: move forward / backward in the base frame
- Left stick vertical: move left / right in the base frame
- Right stick vertical: move upward / downward in the base frame
- `LT` + Left stick horizontal: move forward / backward in the end frame
- `LT` + Left stick vertical: move left / right in the end frame
- `LT` + Right stick vertical: move upward / downward in the end frame
- `Y`: Start gravity compensation. Use any position control commands to exit this mode
- `A`: Start recording. Press again to stop recording
- `B`: Start replaying recorded trajectory
- `RT` + Right stick vertical: rotate pitch relative to end frame
- `RT` + Right stick horizontal: rotate yaw relative to end frame
- `RT` + D-Pad left / right: rotate roll relative to end frame

Please put forward feature requests in the issue sections if you need more features.

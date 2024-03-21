# airbot_ros2_interface

## Nodes

### joy_proxy

Calculate joy stick output, provide topic with contain joy data changes.

#### Subscribe Topics

- joy ([sensor_msgs/msg/Joy](https://docs.ros2.org/latest/api/sensor_msgs/msg/Joy.html)) : from ros2 joy stick driver

#### Advertised Topics

- joy_trigger ([sensor_msgs/msg/Joy](https://docs.ros2.org/latest/api/sensor_msgs/msg/Joy.html)) : differentiated joy stick data.
- joy_latched ([sensor_msgs/msg/Joy](https://docs.ros2.org/latest/api/sensor_msgs/msg/Joy.html)) : original data


### airbot_arm_ros

This is the ros2 interface for airbot, communication with airbot hardware and publish states to ros2 topic / receive commands from ros2 topics.

#### Parameters
Note that parameters are passed through the launch file in *launch* directory.

- *urdf*
  - urdf path for robot description file
  - type: string
  - default value: (airbot_description)/urdf/airbot_play_v2_1.urdf
- *interface*
  - socket-can net device name
  - type: string
  - default value: can0
- *end_mode*
  - type: string
  - default value: teacher

#### Subscribe Topics

- /airbot_play/pose_cmd ([geometry_msgs/msg/Pose](https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html)) : cartesian space control command
- /airbot_play/joint_cmd ([sensor_msgs/msg/JointState](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)) : joint space control command
- /airbot_play/gripper/state_cm ([std_msgs/msg/Bool](https://docs.ros2.org/latest/api/std_msgs/msg/Bool.html)) : simple bool command to control gripper
- /joy_latched ([sensor_msgs/msg/Joy](https://docs.ros2.org/latest/api/sensor_msgs/msg/Joy.html)) : joy stick command, use differentiated data from joy_proxy

#### Advertised Topics

- /airbot_play/arm_pose  ([geometry_msgs/msg/Pose](https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html)) : current robot pose in cartesian coordinates
- /airbot_play/joint_states ([sensor_msgs/msg/JointState](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)) : current robot joint state 
- /airbot_play/gripper/state ([std_msgs/msg/String](https://docs.ros2.org/latest/api/std_msgs/msg/String.html)) : enum for gripper current state, possible value is : `open`, `close`, `moving`

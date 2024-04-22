# SDK
[![Latest Release](https://git.qiuzhi.tech/airbot-play/control/sdk/-/badges/release.svg)](https://git.qiuzhi.tech/airbot-play/control/sdk/-/releases)

[toc]

## 1. Introduction

Here is the project with control programs developed by DISCOVER Robotics for the Airbot Play robotic arm base on code library '[arm-control](https://git.qiuzhi.tech/airbot-play/control/arm-control/-/releases)', offers serveral interface for effortless arm manipulation.

## 2. Programs List

<table border="2">
    <tr>
        <td align="center" width="20%"><strong>Program name</td>
        <td align="center" width="20%"><strong>Interface</td>
        <td align="center" width="60%"><strong>Basic functions</td>
    </tr>
    <tr>
        <td align="center">airbot_kbd_ctrl</td>
        <td align="center">cpp</td>
        <td align="center">Control AIRBOT Play via keyboard</td>
    </tr>
    <tr>
        <td align="center">airbot_sync</td>
        <td align="center">cpp</td>
        <td align="center">Control one AIRBOT Play via following another which being dragged</td>
    </tr>
    <tr>
        <td align="center">airbot_set_zero</td>
        <td align="center">cpp</td>
        <td align="center">Set zero points for the motors of AIRBOT Play</td>
    </tr>
    <tr>
        <td align="center">airbot_auto_set_zero</td>
        <td align="center">cpp</td>
        <td align="center">Auto set zero points for the first three motors of AIRBOT Play</td>
    </tr>
    <tr>
        <td align="center">airbot_demonstrate</td>
        <td align="center">cpp</td>
        <td align="center"></td>
    </tr>
    <tr>
        <td align="center">airbot_demonstrate_dual</td>
        <td align="center">cpp</td>
        <td align="center"></td>
    </tr>
    <tr>
        <td align="center">iap_burn</td>
        <td align="center">python</td>
        <td align="center">Tools to update firmware for first three motors and end board of AIRBOT Play</td>
    </tr>
    <tr>
        <td align="center">joint_speed</td>
        <td align="center">python</td>
        <td align="center">Simple example to control AIRBOT Play by giving joints speed</td>
    </tr>
    <tr>
        <td align="center">joint_position</td>
        <td align="center">python</td>
        <td align="center">Simple example to control AIRBOT Play by giving joints position</td>
    </tr>
    <tr>
        <td align="center">gravity_compensation</td>
        <td align="center">python</td>
        <td align="center">Simple example to control AIRBOT Play in gravity compensation mode</td>
    </tr>
    <tr>
        <td align="center">ros sdk</td>
        <td align="center">ros</td>
        <td align="center">Control AIRBOT Play through rostopic also publish messages </td>
    </tr>
</table>

## 3. Usage
### 3.1 `airbot_kbd_ctrl`
#### 3.1.1 Command line arguments reference

```shell
Usage: airbot_kbd_ctrl [--help] [--version] --master VAR [--node VAR...] [--master-end-mode VAR] [--trajectory VAR] [--direction VAR] [--urdf VAR] [--master-speed VAR] [--constrained] [--forearm-type VAR]

A simple program to control AIRBOT Play via keyboard.

Optional arguments:
  -h, --help             shows help message and exits
  -v, --version          prints version information and exits
  -m, --master           Can device interface of the master arm. [nargs=0..1] [default: "can0"]
  -n, --node             Can device interface of the following arm. Can use multiple times (multiple following arms). E.g., -n can1 can2 [nargs: 0 or more] [default: {}]
  -e, --master-end-mode  The mode of the master arm end effector. Available choices:
                         "teacher": The demonstrator equipped with Damiao motor
                         "gripper": The gripper equipped with Damiao motor
                         "yinshi": The Yinshi two-finger gripper
                         "newteacher": The demonstrator equipped with self-developed motor
                         "none": The arm is not equipped with end effector. [nargs=0..1] [default: "newteacher"]
  -t, --trajectory       The trajectory file to replay [nargs=0..1] [default: ""]
  -d, --direction        The gravity direction. Useful for arms installed vertically [nargs=0..1] [default: "down"]
  -u, --urdf             Manually provided URDF path to override default paths. [nargs=0..1] [default: ""]
  --master-speed         The joint speed of the master arm in ratio of PI. [nargs=0..1] [default: 1]
  --constrained          Stop arm when going out of bounds in gravity compensation mode. False by default
  --forearm-type         The type of forearm. Available choices: "DM": Damiao motor, "OD": Self-developed motors [nargs=0..1] [default: "DM"]
```

#### 3.1.2 Keyboard control reference

##### 3.1.2.1 General Control

<table border="2">
	<tr>
		<td align="left" width="5%"><strong>Action</td>
		<td align="center" width="1%"><strong>Key</td>
		<td align="left" width="30%"><strong>Description</td>
	</tr>
	<tr>
		<td align="left">Stop Program</td>
		<td align="center"><code>z</code> or <code>Ctrl</code> + <code>c</code></td>
		<td align="left">Press 'z' or 'Ctrl+c' to make robot return to zero point and stop the program.
	</tr>
	<tr>
		<td align="left">Switch Base / End Coord</td>
		<td align="center"><code>r</code></td>
		<td align="left">Press 'r' to switch between control in end frame and control in base frame. The default is to control in the base frame </td>
	</tr>
</table>

##### 3.1.2.2 Pose Control

<table border="2">
	<tr>
		<td align="center" width="20%" colspan="3" rowspan="2"><strong>Translation</td>
		<td align="center" width="40%" colspan="3"><strong>Description</td>
		<td align="center" width="20%" colspan="3" rowspan="2"><strong>Rotation</td>
	</tr>
	<tr>
		<td align="center" colspan="3">Press <code>r</code> to switch between control in end frame and control in base frame
	</tr>
	<tr>
		<td align="center" bgcolor="#B0C4DE" rowspan="3"><strong>q</td>
		<td align="center" bgcolor="#FFB6C1" rowspan="3"><strong>w</td>
		<td align="center" bgcolor="#B0C4CE" rowspan="3"><strong>e</td>
		<td align="left" colspan="3"><code>w</code>and <code>s</code> control translation along the <strong>X</strong> axis, where <code>w</code> is positive dirrection.
		</td>
		<td align="center" bgcolor="#B0C4DE" rowspan="3"><strong>u</td>
		<td align="center" bgcolor="#FFB6C1" rowspan="3"><strong>i</td>
		<td align="center" bgcolor="#B0C4DE" rowspan="3"><strong>o</td>
	</tr>
	<tr>
		<td align="left" colspan="3"><code>a</code>and <code>d</code> control translation along the <strong>Y</strong> axis, where <code>a</code> is positive dirrection.
		</td>
	</tr>
	<tr>
		<td align="left" colspan="3"><code>q</code>and <code>e</code> control translation along the <strong>Z</strong> axis, where <code>q</code> is positive dirrection.
		</td>
	</tr>
	<tr>
		<td align="center" bgcolor="#C7EDCC" rowspan="3"><strong>a</td>
		<td align="center" bgcolor="#FFB6C1" rowspan="3"><strong>s</td>
		<td align="center" bgcolor="#C7EDCC" rowspan="3"><strong>d</td>
		<td align="left" colspan="3"><code>i</code>and <code>k</code> control rotation along the <strong>X</strong> axis, where <code>i</code> is positive dirrection.
		</td>
		<td align="center" bgcolor="#C7EDCC" rowspan="3"><strong>j</td>
		<td align="center" bgcolor="#FFB6C1" rowspan="3"><strong>k</td>
		<td align="center" bgcolor="#C7EDCC" rowspan="3"><strong>l</td>
	</tr>
	<tr>
		<td align="left" colspan="3"><code>j</code>and <code>l</code> control rotation along the <strong>Y</strong> axis, where <code>j</code> is positive dirrection.
		</td>
	</tr>
	<tr>
		<td align="left" colspan="3"><code>u</code>and <code>o</code> control rotation along the <strong>Z</strong> axis, where <code>u</code> is positive dirrection.
		</td>
	</tr>
</table>

##### 3.1.2.3 Joint Control

<table border="2">
	<tr>
		<td align="left" width="10%" rowspan="2"><strong>Action</td>
		<td align="center" width="5%" colspan="2"><strong>Key</td>
		<td align="left" width="33%" rowspan="2"><strong>Description</td>
	</tr>
	<tr>
		<td align="center">Add</td>
		<td align="center">Minus</td>
	</tr>
	<tr>
		<td align="left">Joint 1 Control</td>
		<td align="center"><code>1</code></td>
		<td align="center"><code>2</code></td>
		<td align="left">Press the key '1'/'2' to increase/decrease joint 1 angle.</td>
	</tr>
	<tr>
		<td align="left">Joint 2 Control</td>
		<td align="center"><code>3</code></td>
		<td align="center"><code>4</code></td>
		<td align="left">Press the key '3'/'4' to increase/decrease joint 2 angle.</td>
	</tr>
	<tr>
		<td align="left">Joint 3 Control</td>
		<td align="center"><code>5</code></td>
		<td align="center"><code>6</code></td>
		<td align="left">Press the key '5'/'6' to increase/decrease joint 3 angle.</td>
	</tr>
	<tr>
		<td align="left">Joint 4 Control</td>
		<td align="center"><code>7</code></td>
		<td align="center"><code>8</code></td>
		<td align="left">Press the key '7'/'8' to increase/decrease joint 4 angle.</td>
	</tr>
	<tr>
		<td align="left">Joint 5 Control</td>
		<td align="center"><code>9</code></td>
		<td align="center"><code>0</code></td>
		<td align="left">Press the key '9'/'0' to increase/decrease joint 5 angle.</td>
	</tr>
	<tr>
		<td align="left">Joint 6 Control</td>
		<td align="center"><code>-</code></td>
		<td align="center"><code>=</code></td>
		<td align="left">Press the key '-'/'=' to increase/decrease joint 6 angle.</td>
	</tr>
	<tr>
		<td align="left">Gripper</td>
		<td align="center"><code>[</code></td>
		<td align="center"><code>]</code></td>
		<td align="left">Press the key '['/']' to increase/decrease gripper angle.</td>
	</tr>
</table>

#### 3.1.3 Button control reference
<table border='2'>
    <tr>
		<td align="center" width="40%"><strong>Action</td>
		<td align="center" width="20%"><strong>Button</td>
		<td align="center" width="40%"><strong>Description</td>
	</tr>
	<tr>
		<td align="left">Enter gravity compensation mode (when not in)</td>
		<td align="center">Button below joint 6</td>
		<td align="left">Longly click to make robot enter gravity compensation mode.
	</tr>
    <tr>
		<td align="left">Exit gravity compensation mode (when in)</td>
		<td align="center">Button below joint 6</td>
		<td align="left">Longly click to make robot exit gravity compensation mode.
	</tr>
    <tr>
		<td align="left">Enter offline mode (when not in)</td>
		<td align="center">Button below joint 6</td>
		<td align="left">Double click to make robot enter offline mode.
	</tr>
    <tr>
		<td align="left">Exit offline mode (when in)</td>
		<td align="center">Button below joint 6</td>
		<td align="left">Double click to make robot exit offline mode.
	</tr>
    <tr>
		<td align="left">Switch end gripper state (open or closed) (disabled in offline mode)</td>
		<td align="center">Button below joint 6</td>
		<td align="left">Shortly click to open or closed gripper.
	</tr>
    <tr>
		<td align="left">Start/stop path record (only enable in gravity compensation mode)</td>
		<td align="center">Button on base control board</td>
		<td align="left">Shortly click to start or stop path record in gravity compensation mode.
	</tr>
    <tr>
		<td align="left">Start path replay (only enable in offline mode)</td>
		<td align="center">Button on base control board</td>
		<td align="left">Shortly click to start path replay.
	</tr>
</table>

### 3.2 `airbot_sync`
```shell
Usage: airbot_sync [--help] [--version] [--urdf VAR] --master VAR --node VAR [--direction VAR] [--master-end-mode VAR] [--follower-end-mode VAR] [--master-speed VAR] [--follower-speed VAR] [--force-feedback]

Optional arguments:
  -h, --help           shows help message and exits
  -v, --version        prints version information and exits
  -u, --urdf           Manually provided URDF path to override default paths. [nargs=0..1] [default: ""]
  -m, --master         Can device interface of the master arm. [nargs=0..1] [default: "can0"]
  -n, --node           Can device interface of the following arm. [nargs=0..1] [default: "can1"]
  -d, --direction      The gravity direction. Useful for arms installed vertically [nargs=0..1] [default: "down"]
  --master-end-mode    The mode of the master arm end effector. Available choices: "teacher", "gripper", "yinshi", "newteacher" [nargs=0..1] [default: "newteacher"]
  --follower-end-mode  The mode of the follower arm end effector. Available choices: "teacher", "gripper", "yinshi", "newteacher" [nargs=0..1] [default: "gripper"]
  --master-speed       The joint speed of the master arm in percentage of PI. [nargs=0..1] [default: 3.14159]
  --follower-speed     The joint speed of the follower arm in percentage of PI. [nargs=0..1] [default: 9.42478]
  --force-feedback     Enable force feedback control.
```
### 3.3 `airbot_set_zero`
```bash
Usage: airbot_set_zero [--help] [--version] --master VAR [--master-end-mode VAR] [--forearm-type VAR]

This is a tool to set zero points for the motors of AIRBOT Play. This tool should only be used during manufacturing and maintenance. It is not recommended to use this tool in normal operation
If an end effector is attached, it is important to keep the end effector in zero position during the process.

Optional arguments:
  -h, --help             shows help message and exits
  -v, --version          prints version information and exits
  -m, --master           Can device interface of the master arm. Default: can0 [nargs=0..1] [default: "can0"]
  -e, --master-end-mode  The mode of the master arm end effector. Available choices:
                         "teacher": The demonstrator equipped with Damiao motor
                         "gripper": The gripper equipped with Damiao motor
                         "yinshi": The Yinshi two-finger gripper
                         "newteacher": The demonstrator equipped with self-developed motor
                         "none": The arm is not equipped with end effector. [nargs=0..1] [default: "none"]
  --forearm-type         The type of forearm. Available choices: "DM": Damiao motor, "OD": Self-developed motors [nargs=0..1] [default: "DM"]
```
### 3.4 `airbot_auto_set_zero`
```shell
Usage: airbot_auto_set_zero [--help] [--version] --master VAR [--urdf VAR] [--forearm-type VAR]

A simple program to set zero point of AIRBOT Play.

Optional arguments:
  -h, --help      shows help message and exits
  -v, --version   prints version information and exits
  -m, --master    Can device interface of the master arm. [nargs=0..1] [default: "can0"]
  -u, --urdf      Manually provided URDF path to override default paths. [nargs=0..1] [default: ""]
  --forearm-type  The type of forearm. Available choices: "DM": Damiao motor, "OD": Self-developed motors [nargs=0..1] [default: "DM"]
```
### 3.5 `airbot_demonstrate`
```shell
Usage: airbot_demonstrate [--help] [--version] [--urdf VAR] --master VAR --node VAR [--direction VAR] [--master-end-mode VAR] [--follower-end-mode VAR] [--master-speed VAR] [--follower-speed VAR] [--frequency VAR] [--camera VAR]... [--start-episode VAR] [--task-name VAR] [--max-time-steps VAR] [--start-joint-pos VAR...]

Optional arguments:
  -h, --help               shows help message and exits
  -v, --version            prints version information and exits
  -u, --urdf               Manually provided URDF path to override default paths. [nargs=0..1] [default: ""]
  -m, --master             Can device interface of the master arm. [nargs=0..1] [default: "can0"]
  -n, --node               Can device interface of the following arm. [nargs=0..1] [default: "can1"]
  -d, --direction          The gravity direction. Useful for arms installed vertically [nargs=0..1] [default: "down"]
  --master-end-mode        The mode of the master arm end effector. Available choices: "teacher", "gripper", "yinshi", "newteacher" [nargs=0..1] [default: "newteacher"]
  --follower-end-mode      The mode of the follower arm end effector. Available choices: "teacher", "gripper", "yinshi", "newteacher" [nargs=0..1] [default: "gripper"]
  --master-speed           The joint speed of the master arm in percentage of PI. [nargs=0..1] [default: 1]
  --follower-speed         The joint speed of the follower arm in percentage of PI. [nargs=0..1] [default: 3]
  -f, --frequency          The frequency of the recording action [nargs=0..1] [default: 15]
  -c, --camera             Camera devices indices. Can use multiple times (multiple cameras). E.g., -c 0 -c 1 [nargs=0..1] [default: {}] [may be repeated]
  -se, --start-episode     The start episode number for saving data. Default is 0. [nargs=0..1] [default: 0]
  -tn, --task-name         The name of the task which will be used as the folder name. [nargs=0..1] [default: "test_task"]
  -mts, --max-time-steps   The max time steps to collect data. [nargs=0..1] [default: 1000]
  -sjp, --start-joint-pos  Start joint positions for data collection [nargs: 7] [default: {0 0 0 0...0}]
```

### 3.6 `airbot_demonstrate_dual`
```bash
Usage: airbot_demonstrate_dual [--help] [--version] [--urdf VAR] --master_left VAR --node_left VAR --master_right VAR --node_right VAR [--direction VAR] [--master-end-mode VAR] [--frequency VAR] [--follower-end-mode VAR] [--master-speed VAR] [--follower-speed VAR] [--camera VAR]... [--start-episode VAR] [--task-name VAR] [--max-time-steps VAR] [--start-joint-pos-left VAR...] [--start-joint-pos-right VAR...]

Optional arguments:
  -h, --help                      shows help message and exits
  -v, --version                   prints version information and exits
  -u, --urdf                      URDF file for describing the arm. If not provided, check and read from the environment variable URDF_PATH or load from deb install path. [nargs=0..1] [default: "/usr/local/share/airbot_play/airbot_play_v2_1/urdf/airbot_play_v2_1_with_gripper.urdf"]
  -ml, --master_left              Can device interface of the master arm. [nargs=0..1] [default: "can0"]
  -nl, --node_left                Can device interface of the following arm. [nargs=0..1] [default: "can1"]
  -mr, --master_right             Can device interface of the master arm. [nargs=0..1] [default: "can2"]
  -nr, --node_right               Can device interface of the following arm. [nargs=0..1] [default: "can3"]
  -d, --direction                 The gravity direction. Useful for arms installed vertically [nargs=0..1] [default: "down"]
  --master-end-mode               The mode of the master arm end effector. Available choices: "teacher", "gripper", "yinshi", "newteacher" [nargs=0..1] [default: "newteacher"]
  -f, --frequency                 The frequency of the recording action [nargs=0..1] [default: 15]
  --follower-end-mode             The mode of the follower arm end effector. Available choices: "teacher", "gripper", "yinshi", "newteacher" [nargs=0..1] [default: "gripper"]
  --master-speed                  The joint speed of the master arm in percentage of PI. [nargs=0..1] [default: 1]
  --follower-speed                The joint speed of the follower arm in percentage of PI. [nargs=0..1] [default: 3]
  -c, --camera                    Camera devices indices. Can use multiple times (multiple cameras). E.g., -c 0 -c 1 [nargs=0..1] [default: {}] [may be repeated]
  -se, --start-episode            The start episode number for saving data. Default is 0. [nargs=0..1] [default: 0]
  -tn, --task-name                The name of the task which will be used as the folder name. [nargs=0..1] [default: "test_task"]
  -mts, --max-time-steps          The max time steps to collect data. [nargs=0..1] [default: 1000]
  -sjpl, --start-joint-pos-left   Start joint positions for data collection [nargs: 7] [default: {0 0 0 0...0}]
  -sjpr, --start-joint-pos-right  Start joint positions for data collection [nargs: 7] [default: {0 0 0 0...0}]
```
### 3.7 iap_burn
```shell
usage: iap_burn.py [-h] [-i CAN_ID] [-m CAN_INTERFACE] [-n DEVICE_NAME] firmware_path

IAP burn tool

positional arguments:
  firmware_path         Firmware path

optional arguments:
  -h, --help            show this help message and exit
  -i CAN_ID, --can-id CAN_ID
  -m CAN_INTERFACE, --can-interface CAN_INTERFACE
                        CAN interface
  -n DEVICE_NAME, --device-name DEVICE_NAME
                        Device name: arm-interface-board-end or vesc-motor-control
```

### 3.8 ROS SDK
```bash
roslaunch ros_interface airbot_arm.launch
```

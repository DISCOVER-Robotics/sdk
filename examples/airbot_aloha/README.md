# AIRBOT Play × ALOHA

## Introduction

This tutorial is about demonstration (single or double arm manipulation) for raw data collecting, data converting (convert raw data to hdf5 file used by the learning algorithms) and data replaying.

Please ask the customer service for all available dependency packages and source codes files. We will gradually provide relevant download links in the future.

> **Docker Notation**
>
> If you want to install and use these packages in Docker, you should run your container with some necessary args, for example:
>
> ```bash
> docker run -it --name airbot_play --network=host --privileged=true -v /lib/modules:/lib/modules,/tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY ubuntu:20.04
> ```
> And you probably can not install the deb packages when they are in `/root`. In this case, you can move the packages and the terminal to `/tmp` first:
>
> ```bash
> mv airbot_*.deb /tmp/ && cd /tmp
> ```
>
> In some Docker Containers, there is no `sudo` command, you can just remove it from the commands below or you can install it by running: `apt install sudo`.

## Environment Setup

!!! tip "Supported Operation Systems"

    * Ubuntu 20.04 LTS AMD64(x86_64)
    * Ubuntu 20.04 LTS ARM64

### Data Collection and Convertion Environment

#### Basic Setup

The core control package for data collection should be in the format of `airbot_play_<version>_<arch>.deb`. Install this package with the following command:

```bash
sudo apt update
sudo apt install python3-pip python3 udev kmod iproute2 libcanberra-gtk-module libcanberra-gtk3-module liburdf-dev libspdlog-dev libfmt-dev liborocos-kdl1.4 -y
sudo service udev restart && udevadm control --reload
sudo apt install ./airbot_play_*.deb -y
```

The functional package for data collection should be in the format of `airbot_aloha_<version>_<arch>.deb`. Install this package with the following command:

```bash
sudo apt install ./airbot_aloha_*.deb
```

#### Mobile Setup (optional)

If you also want to collect mobile data, please refer to [airbase repo](https://github.com/RoboticsChen/airbase) for environment setup and basic usage (make sure the python packages are installed in the same env as others).

Then you can use the package for data collection together with airbase in the format of `airbot_aloha_mobile_<version>_<arch>.deb`. If you have installed `airbot_aloha_<version>_<arch>.deb`, it will be replaced.

```bash
sudo apt install ./airbot_aloha_mobile_*.deb
```

### Data Replay Environment

>- [Data Collection and Convertion Environment](#data-collection-and-convertion-environment) needs to be configured first.
>
>- This configuration will install the AIRBOT Play Python API.

The source code file name should be in the format of `airbot_play_python_<version>.zip`. Run the following command to extract the zip package:

```bash
cp airbot_play_python_*.zip airbot_play_python.zip
unzip airbot_play_python.zip -q && sudo rm -rf airbot_play_python.zip
```

Then install the package (using `/usr/bin/python3` command since the `airbot_aloha` package is installed by it):

```bash
sudo apt install librosconsole-dev git -y
cd airbot_play_python/python
git clone --depth 1 https://github.com/pybind/pybind11.git
/usr/bin/python3 setup.py install
```

> Note: If you manually download the pybind11 project, then you should extract and rename the folder to `pybind11` and replace the empty pybind11 folder in `airbot_play_python/python`.

## Data Collection

> * [Data Collection and Convertion Environment](#data-collection-and-convertion-environment) needs to be configured first.
>
> * The task name should be reasonable, and it is recommended to include time in the name to distinguish the same task data collected at different times.

### Starting Robotic Arms
1. Prepare all teaching arms and execution robotic arms.
2. Connect the power sources of all robotic arms (order doesn't matter).
3. **First**, connect the teaching arm via Type-C data cable (corresponding to CAN0), **then** connect the execution arm, too (corresponding to CAN1). For dual-arm operations, follow the above sequence for the left-side robotic arm first, then the right-side arm.
4. Long-press the power button on each robotic arm to turn them on.
5. Ensure that the robotic arms are at the zero pose; otherwise, perform a [zero calibration](https://discover-robotics.github.io/docs/manual/#_11).

> Note: Other devices connected to your computer may occupy the CAN interfaces, you may need to change the default can interfaces manually. Please refer to [Explanation of Parameters](#explanation-of-parameters).

### Connecting Cameras
Data collection typically requires multiple cameras, and the connection order can be as follows:

- Single-arm task sequence: Arm-mounted camera left eye -> Arm-mounted camera right eye -> Base-mounted camera
- Dual-arm task: Left arm camera -> Right arm camera -> Environment camera top -> Environment camera bottom

The above sequence is just a reference: the actual connection order depends on the number of cameras and their placement. Generally, prioritize connecting arm-mounted, left-side, and overhead cameras first.

### Starting Robotic Base (for mobile task)

Press and hold the power button of the robot base until it lights up. Wait for the robot base WIFI starting and then connect to it. Make sure the emergency button and brake button are both not pressed. Please refer to [airbase repo](https://github.com/RoboticsChen/airbase?tab=readme-ov-file#starting-the-robot-base) for more details.

### Starting Data Collection

```bash
airbot_demonstrate \
    -c <cam_device_0> <cam_device_1> <cam_device_2> \
    -mts <max_time_step> \
    -tn <task_name> \
    -se <start_episode_index> \
    -f 15 \
    -gn 1\
    -sjp <joint_pos_1> <joint_pos_2> <joint_pos_3> <joint_pos_4> <joint_pos_5> <joint_pos_6> <gripper_pos>
```
You can specify the paramter '-gn' to choose to collect with single arm (default) or dual arm (-gn 2) .

#### Explanation of Parameters

- `-c`: Device number of the USB cameras, the order of multiple device numbers (i.e., camera sequence) is specified by the specific task.
    - Typically, follow the connection order, where available camera device numbers are incremented by even numbers, such as 0, 2, 4...
    - If your computer has a built-in regular camera, it occupies one device number, and externally connected camera device numbers start from 2 and increment by even numbers: 2, 4, 6... (if cameras were connected before the computer started, the device number of the built-in camera may not be 0; in rare cases, after connecting cameras post-startup, the built-in camera's device number may not be 0).
    - If your computer has a built-in depth/structured light camera (like Windows Hello camera), the RGB camera and depth camera each occupy a device number, and externally connected camera device numbers start from 4 and increment by even numbers: 4, 6, 8...
- `-can`: Can device interfaces of the teach-follow arms groups, default is '0 1 2 3', in this case, can0&can2 correspond to the teacher arms, and can1&can3 correspond to the follower arms.
- `-mts`: Specifies the maximum number of frames to be captured; specified by the specific task.
When reaching the maximum time steps, the program will prompt.
- `-tn`: Task name, specified by the specific task.
    - The collected data will be saved in the `demonstrations/(raw/)<task_name>` folder in the current directory.
- `-se`: Starting episode number for collection, default is 0.
    - After interruption, you can modify this value to continue collection without overwriting previous data.
    - When data is mistakenly saved, specifying this episode number allows for re-collection to overwrite existing data.
    - When collecting data multiple times after executing the command, the episode numbers of subsequent collections will increment from this base.
    - Each set of data is saved in the `demonstrations/(raw/)<task_name>/<episode_id>` folder.
- `-sjp`: Initial positions of each joint and gripper before starting collection for each episode; defaults to `0 0 0 0 0 0 0`. If you use dual arms and want them to have different initial joint angles, you can append values back, e.g. `0 0 0 0 0 0 0 0 0 0 0 0 0 0`.
- `-f`: Data collection frequency, default is 15Hz.
- `-gn`: The number of robot teach-follow group to execute in the system, default is 1.

For mobile usage, there are some other parameters you can set:

- `-mn`: The name of the map to build/load that is used to move the base to a absolute pose.
- `-sp`: The move speed of the RobotBase when moving to a target pose. The value can be low, medium, high or float value ∈ (0, 1.5].

#### Excution Example

- One teacher with one follower:
    ```bash
    airbot_demonstrate -c 0 -mts 100 -tn example_task -sjp 0.0 0.0 0.0 0.0 0.0 0.0 0.0
    ```
- Two teachers with two followers:
    ```bash
    airbot_demonstrate -c 0 2 -mts 200 -tn example_task -gn 2 -sjp 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
    ```

#### Key Descriptions

> * Do not press or hold keys continuously; otherwise, the key's behavior will repeat sequentially.
>
> * For mobile usage, you can see an additional step information in the terminal after executing the command above.
>  > * If you haven't built any maps in the current environment or the previous map works wrong, you need to press `1` to create a new one. After pressing, the terminal will prompt you to move the robot base to a desired origin position, and then press `Enter` again at that position, and then you can push the robot base around the environment to build the map. Then press `s` to save the map and press `o` to move the robot base to origin.
> > * If you have a good map, you can press 2 to load it and the robot base will take some time to localize and then it will auto move to the origin.
> > * If you just want to use the robotic arms immediately, you can press `3` to skip this step but the pose control of the robot base may work wrong.

After excuting the command above, you can use the keyboard of your computer to control. The key descriptions are as follows:

- `g`: Toggle teach mode on/off.
- `Spacebar`: Start/stop episode data recording.
- `q`: Discard the current record.
- `0`: Return the robotic arm to the initial position.
- `p`: Print current robot states information.

#### Operational Steps

1. Start the program, and the real-time windows of each camera will appear (if not all cameras start, try adjusting device numbers or checking camera connections; try connecting only one camera per docking station; some computers may support only 1-2 external cameras when USB ports share the same bus, consider changing the computer).
2. Press `Spacebar` to start recording data and simultaneously, teleoperate the robotic arm to complete the task.
3. After completing the task, wait to collect the specified number of frames (the number of frames used to complete the task should be as close as possible to the maximum collection frames):
    - If the teaching opration is not acceptable, press `q` to discard the current teaching record, then press `0` to control the robotic arm to return to the initial position.
    - If it is acceptable, press `Spacebar` to save the current teaching record. After saving, the robotic arm will automatically return to the initial position.
4. (Optional) In the `demonstrations/(raw/)<task_name>` folder in the current directory, check the recorded episodes.
    - Each collected episode data includes: videos recorded by cameras (.avi), and a robotic arm status record file (records.json) and a robot base status record file (base_poses.json) if used.

**Additional Notes:**

1. Try to ensure that the completed task actions are completed **just before reaching the maximum frame count**, i.e., do not end the action too early.

2. The robotic arm movement speed should not be too fast; otherwise, the collected information will be sparse, and the image quality will not be high.

3. It is recommended to store the collected task data folder <task_name> in the same directory structure on a **portable hard drive** as a backup.

## Data Convertion
> [Data Collection and Convertion Environment](#data-collection-and-convertion-environment) needs to be set up first.

In the same path where the data collection command was executed, run the following command to save the data in hdf5 format:

```bash
/usr/bin/python3 -m airbot_aloha.convert_episodes -rn 1 -cn 0 -tn example_task -se 0 -ee 0 -rd ./demonstrations
```

**Parameter explanation:**

- `-cn`: Specify camera names. For example, if there are 2 cameras, then specify `-cn 0,1`.
- `-tn`: Specify the task name, same as specified during data collection.
- `-se`: Specify the starting episode number of the data.
- `-ee`: Specify the ending episode number of the data.
- `-rn`: For dual-arm tasks, specify `-rn` 2.
- `-rd`: Path to your data directory.

This command will search for the specified task data in the `demonstrations(/raw)` folder in the current directory and convert them to hdf5 files into the `demonstrations/hdf5` folder.

Similarly, it is recommended to store the converted task data folder `<task_name>` in the same directory structure on a portable hard drive as a backup.

## Data Replay (Optional)

> Data replay can be used to verify if there are issues with collected data, init states of the environment, etc. (requires prior setup of the [Data Replay Environment](#data-replay-environment)).

The data replay command and its parameters are as follows:

```bash
/usr/bin/python3 -m airbot_aloha.replay_episodes -rn 1 -tn example_task -ei 0 -ii -can 1 -cn 0
```

**Parameter explanation:**

- `-tn`: Specify the task name.
- `-ei`: Specify the ID corresponding to the HDF file.
- `-can`: CAN ID of the follower arms. For dual-arm tasks, you can set `-can 1 3` if you followed the connection order in [Starting-Robotic-Arms](#starting-robotic-arms). Also, you can unplug all the Type-C wires first and only connect the follower arms, then set `-can 0 1`.
- `-ii`: Do not replay camera data.
- `-ia`: Do not replay action data.
- `-rn`: For dual-arm tasks, specify `-rn` 2.
- `-cn`: The name of the cameras data to be replayed. For example, if there are two cameras, specify `-cn 0 1`.

For mobile usage, there are some other parameters:

- `-mt`: specify `-mt slantec_athena` to replay the base data at the same time.
- `-bs`: speed of the robot base, the value can be `high`, `medium`, `low` or float number between `(0, 1.5)`.

After executing the command above, you can see the instruction in the terminal (also an additional map options step for mobile usage).

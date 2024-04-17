/*
  This file contains docstrings for use in the Python bindings.
  Do not edit! They were automatically extracted by pybind11_mkdoc.
 */

#define __EXPAND(x)                                      x
#define __COUNT(_1, _2, _3, _4, _5, _6, _7, COUNT, ...)  COUNT
#define __VA_SIZE(...)                                   __EXPAND(__COUNT(__VA_ARGS__, 7, 6, 5, 4, 3, 2, 1, 0))
#define __CAT1(a, b)                                     a ## b
#define __CAT2(a, b)                                     __CAT1(a, b)
#define __DOC1(n1)                                       __doc_##n1
#define __DOC2(n1, n2)                                   __doc_##n1##_##n2
#define __DOC3(n1, n2, n3)                               __doc_##n1##_##n2##_##n3
#define __DOC4(n1, n2, n3, n4)                           __doc_##n1##_##n2##_##n3##_##n4
#define __DOC5(n1, n2, n3, n4, n5)                       __doc_##n1##_##n2##_##n3##_##n4##_##n5
#define __DOC6(n1, n2, n3, n4, n5, n6)                   __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6
#define __DOC7(n1, n2, n3, n4, n5, n6, n7)               __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6##_##n7
#define DOC(...)                                         __EXPAND(__EXPAND(__CAT2(__DOC, __VA_SIZE(__VA_ARGS__)))(__VA_ARGS__))

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif


static const char *__doc_arm_MotorDriver = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorBoundary = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorCommunicationDetect = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorControlMode_e = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorControlMode_e_MIT = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorControlMode_e_NONE = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorControlMode_e_POS = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorControlMode_e_SPD = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorCreate = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorCurrentDetect = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorDeInit =
R"doc(Deinitializes the motor.

This function deinitializes the motor. It performs cleanup and
releases resources associated with motor control.)doc";

static const char *__doc_arm_MotorDriver_MotorDriver = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorDriver_2 = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorErrorDetect = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorErrorModeCmd = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorErrorType_e = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorErrorType_e_COMMUNICATION_ERROR = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorErrorType_e_NONE_ERROR = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorErrorType_e_OVER_CURRENT = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorErrorType_e_OVER_TEMPERATURE = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorGetParam =
R"doc(Requests motor parameters based on a specific command.

This function sends a request to retrieve specific parameters from the
motor. The parameter to be retrieved is identified by the `param_cmd`
argument.

Parameter ``param_cmd``:
    The command code specifying which parameter to retrieve.)doc";

static const char *__doc_arm_MotorDriver_MotorInit =
R"doc(Initializes the motor.

This function initializes the motor for operation. It performs
necessary setup and configuration for motor control.

Returns:
    True if motor initialization is successful; otherwise, false.)doc";

static const char *__doc_arm_MotorDriver_MotorLock =
R"doc(Locks the motor to prevent movement.

This function locks the motor to prevent any movement. Once locked,
the motor will not respond to commands for movement.)doc";

static const char *__doc_arm_MotorDriver_MotorMitModeCmd =
R"doc(Commands the motor to operate in impedance mode with specific
parameters.

This function sets the motor to operate in an impedance control mode,
where it applies force based on the provided parameters.

Parameter ``f_p``:
    Proportional force value.

Parameter ``f_v``:
    Velocity-based force value.

Parameter ``f_kp``:
    Proportional stiffness coefficient.

Parameter ``f_kd``:
    Damping coefficient.

Parameter ``f_t``:
    Desired torque value.)doc";

static const char *__doc_arm_MotorDriver_MotorPosModeCmd =
R"doc(Commands the motor to move to a specified position at a specified
speed.

This function is responsible for commanding the motor to move to a
desired position with a specified speed.

Parameter ``pos``:
    The target position to move the motor to.

Parameter ``spd``:
    The speed at which the motor should move to the target position.

Parameter ``ignore_limit``:
    If true, ignores any position limits that may be set.)doc";

static const char *__doc_arm_MotorDriver_MotorResetID = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorSetFilterParam =
R"doc(Sets the filter parameters for position and speed control.

This function configures the filter parameters used in position and
speed control.

Parameter ``position_kd_filter``:
    Filter coefficient for position control derivative term.

Parameter ``kd_spd``:
    Filter coefficient for speed control derivative term.)doc";

static const char *__doc_arm_MotorDriver_MotorSetPosParam =
R"doc(Sets the position control parameters (proportional gain and derivative
gain) for the motor.

This function configures the position control parameters (PID gains)
for the motor.

Parameter ``kp``:
    Proportional gain for position control.

Parameter ``kd``:
    Derivative gain for position control.)doc";

static const char *__doc_arm_MotorDriver_MotorSetSpdParam =
R"doc(Sets the speed control parameters (proportional gain and integral
gain) for the motor.

This function configures the speed control parameters (PI gains) for
the motor.

Parameter ``kp``:
    Proportional gain for speed control.

Parameter ``ki``:
    Integral gain for speed control.)doc";

static const char *__doc_arm_MotorDriver_MotorSetZero =
R"doc(Sets the motor position to zero.

This function sets the current motor position to zero. It establishes
a new reference point for position measurement.

Returns:
    True if setting motor position to zero is successful; otherwise,
    false.)doc";

static const char *__doc_arm_MotorDriver_MotorSpdModeCmd =
R"doc(Commands the motor to rotate at a specified speed.

This function commands the motor to rotate at the specified speed.

Parameter ``spd``:
    The speed at which the motor should rotate.)doc";

static const char *__doc_arm_MotorDriver_MotorTemperatureDetect = R"doc()doc";

static const char *__doc_arm_MotorDriver_MotorUnlock =
R"doc(Unlocks the motor to allow movement.

This function unlocks the motor to enable movement. After unlocking,
the motor can respond to movement commands.)doc";

static const char *__doc_arm_MotorDriver_MotorWriteFlash = R"doc()doc";

static const char *__doc_arm_MotorDriver_error_id = R"doc()doc";

static const char *__doc_arm_MotorDriver_gear_ratio = R"doc()doc";

static const char *__doc_arm_MotorDriver_get_error_id =
R"doc(Retrieves the error ID associated with the motor.

This function returns the error ID that indicates any error condition
of the motor.

Returns:
    The error ID of the motor.)doc";

static const char *__doc_arm_MotorDriver_get_gear_ratio =
R"doc(Retrieves the gear ratio of the motor.

This function returns the gear ratio of the motor.

Returns:
    The gear ratio of the motor.)doc";

static const char *__doc_arm_MotorDriver_get_max_current = R"doc()doc";

static const char *__doc_arm_MotorDriver_get_motor_acceleration =
R"doc(Retrieves the acceleration of the motor.

This function returns the current acceleration of the motor.

Returns:
    The acceleration of the motor.)doc";

static const char *__doc_arm_MotorDriver_get_motor_control_mode =
R"doc(Retrieves the control mode of the motor.

This function returns the current control mode of the motor.

Returns:
    The control mode of the motor.)doc";

static const char *__doc_arm_MotorDriver_get_motor_current =
R"doc(Retrieves the current current (electric current) of the motor.

This function returns the electric current flowing through the motor.

Returns:
    The current (electric current) of the motor.)doc";

static const char *__doc_arm_MotorDriver_get_motor_error_id =
R"doc(Retrieves the error ID associated with the motor.

This function returns the error ID that indicates any error condition
of the motor.

Returns:
    The error ID of the motor.)doc";

static const char *__doc_arm_MotorDriver_get_motor_id =
R"doc(Retrieves the ID of the motor.

This function returns the unique identifier (ID) of the motor.

Returns:
    The ID of the motor.)doc";

static const char *__doc_arm_MotorDriver_get_motor_kd_pos = R"doc()doc";

static const char *__doc_arm_MotorDriver_get_motor_kd_spd = R"doc()doc";

static const char *__doc_arm_MotorDriver_get_motor_ki_pos = R"doc()doc";

static const char *__doc_arm_MotorDriver_get_motor_ki_spd = R"doc()doc";

static const char *__doc_arm_MotorDriver_get_motor_kp_pos = R"doc()doc";

static const char *__doc_arm_MotorDriver_get_motor_kp_spd = R"doc()doc";

static const char *__doc_arm_MotorDriver_get_motor_pos =
R"doc(Retrieves the current position of the motor.

This function returns the current position of the motor.

Returns:
    The current position of the motor.)doc";

static const char *__doc_arm_MotorDriver_get_motor_spd =
R"doc(Retrieves the current speed of the motor.

This function returns the current speed of the motor.

Returns:
    The current speed of the motor.)doc";

static const char *__doc_arm_MotorDriver_get_motor_temperature =
R"doc(Retrieves the temperature of the motor.

This function returns the current temperature of the motor.

Returns:
    The temperature of the motor.)doc";

static const char *__doc_arm_MotorDriver_get_response_count =
R"doc(Retrieves the count of responses received from the motor.

This function returns the number of responses received from the motor.

Returns:
    The count of responses received from the motor.)doc";

static const char *__doc_arm_MotorDriver_get_timeout =
R"doc(Retrieves the timeout value configured for the motor.

This function returns the timeout value set for the motor operations.

Returns:
    The timeout value in milliseconds.)doc";

static const char *__doc_arm_MotorDriver_get_write_para_res =
R"doc(Retrieves the result of writing parameters to the motor.

This function returns the result of writing parameters to the motor.

Returns:
    True if writing parameters was successful; otherwise, false.)doc";

static const char *__doc_arm_MotorDriver_get_write_para_res_and_clear = R"doc()doc";

static const char *__doc_arm_MotorDriver_heartbeat_detect_counter = R"doc()doc";

static const char *__doc_arm_MotorDriver_logger = R"doc()doc";

static const char *__doc_arm_MotorDriver_max_current = R"doc()doc";

static const char *__doc_arm_MotorDriver_motor_acceleration = R"doc()doc";

static const char *__doc_arm_MotorDriver_motor_control_mode = R"doc()doc";

static const char *__doc_arm_MotorDriver_motor_current = R"doc()doc";

static const char *__doc_arm_MotorDriver_motor_kd_pos = R"doc()doc";

static const char *__doc_arm_MotorDriver_motor_kd_spd = R"doc()doc";

static const char *__doc_arm_MotorDriver_motor_ki_pos = R"doc()doc";

static const char *__doc_arm_MotorDriver_motor_ki_spd = R"doc()doc";

static const char *__doc_arm_MotorDriver_motor_kp_pos = R"doc()doc";

static const char *__doc_arm_MotorDriver_motor_kp_spd = R"doc()doc";

static const char *__doc_arm_MotorDriver_motor_pos = R"doc()doc";

static const char *__doc_arm_MotorDriver_motor_spd = R"doc()doc";

static const char *__doc_arm_MotorDriver_motor_temperature = R"doc()doc";

static const char *__doc_arm_MotorDriver_set_max_current = R"doc()doc";

static const char *__doc_arm_MotorDriver_set_motor_control_mode =
R"doc(Sets the control mode for the motor.

This function specifies the control mode for the motor.

Parameter ``motor_control_mode``:
    The control mode to be set for the motor.)doc";

static const char *__doc_arm_MotorDriver_set_motor_id =
R"doc(Sets the ID of the motor.

This function assigns a unique identifier (ID) to the motor.

Parameter ``motor_id``:
    The ID of the motor.)doc";

static const char *__doc_arm_MotorDriver_timeout = R"doc()doc";

static const char *__doc_arm_MotorDriver_write_para_res = R"doc()doc";

static const char *__doc_arm_Robot =
R"doc(The Robot class for controlling the robot arm

1. plug-to-play 2. control mode and light effect 3. Demonstrate Mode /
Replay Mode / Online Mode)doc";

static const char *__doc_arm_Robot_ArmMode = R"doc()doc";

static const char *__doc_arm_Robot_ArmMode_GRAVITY_COMPENSATION = R"doc()doc";

static const char *__doc_arm_Robot_ArmMode_NORMAL = R"doc()doc";

static const char *__doc_arm_Robot_ArmMode_OFFLINE = R"doc()doc";

static const char *__doc_arm_Robot_ArmMode_RECORD = R"doc()doc";

static const char *__doc_arm_Robot_MotorControlState = R"doc()doc";

static const char *__doc_arm_Robot_MotorControlState_INIT = R"doc()doc";

static const char *__doc_arm_Robot_MotorControlState_JOINT_POS = R"doc()doc";

static const char *__doc_arm_Robot_MotorControlState_JOINT_TORQUE = R"doc()doc";

static const char *__doc_arm_Robot_MotorControlState_JOINT_VEL = R"doc()doc";

static const char *__doc_arm_Robot_Robot =
R"doc(Construct a new Robot object

Once created, the instance will perform the following actions:

1. initialize forward and inverse kinematics solvers 2. initialize
text-based loggers; logs will be saved in the `logs` directory 3.
initialize and check the status of the base interface board 4.
initialize and check the status of the motors, from the base to the
end 5. initialize and check the status of the end interface board 6.
update motor status 7. start the following threads: *
thread_sync_pose_: map joints to end pose * thread_update_motor_: send
CAN messages to motors * thread_plan_: perform planning * thread_log_:
start pushing robot status to Kibana * thread_snap_: start listening
to the snap signals. will be removed in the future 8. unlock the
motors

Typically, the instance should be created once and used throughout the
program. Also, it is the user's responsibility to ensure that **ONLY
ONE** instance is controlling the robot at a time.

Parameter ``urdf_path``:
    the path to valid AIRBOT Play URDF File. By default, when
    installed via apt package, two valid urdf files are installed: -
    AIRBOT Play with no end effector: `/usr/local/share/airbot_play/ai
    rbot_play_v2_1/urdf/airbot_play_v2_1.urdf` - AIRBOT Play with
    AIRBOT Play demonstrator: `/usr/local/share/airbot_play/airbot_pla
    y_v2_1/urdf/airbot_play_v2_1_with_gripper.urdf`

Parameter ``can_interface``:
    the interface recognized by the system. Usually in the form like
    `can0`, `can1`, etc. The currently available interfaces can be
    found by `ip link` command, given that `iproute2` package is
    installed on Debian-based systems

Parameter ``direction``:
    the direction of the gravity. If AIRBOT Play is installed on a
    vertical surface, this option should be altered. Available
    options: "down", "left", "right"

Parameter ``vel``:
    the maximum velocity of joints in Online / Replay mode.

Parameter ``end_mode``:
    the end effector installed at the end. Available options: -
    `none`: no end effector is installed - `gripper`: AIRBOT Gripper
    is installed - `newteacher`: AIRBOT Demonstrator is installed

Parameter ``constraint``:
    whether or not AIRBOT Play stop moving once outside the joint
    limits

Parameter ``ignore_status_check``:
    whether or not AIRBOT Play ignore the status check. This option is
    DEPRECATED, as it will be removed in the future

Parameter ``forearm_tpe``:
    the type of the forearm. Available options: - `DM`: Damiao motors
    are installed - `OD`: Self-designed motors are installed

Parameter ``stay_init``:
    whether or not AIRBOT Play stay in the initial position. If
    `false`, AIRBOT Play will move to the zero position once
    initialized. This option is DEPRECATED, as it will be removed in
    the future

Parameter ``ignore_limit``:
    whether or not AIRBOT Play ignore the joint limits. This option is
    DEPRECATED, as it will be removed in the future)doc";

static const char *__doc_arm_Robot_add_target_joint_q =
R"doc(Position control method. Set the target joint positions of the robot
arm in joint space. The robot will move to the target pose by adding
the relative joint positions to the current planning target joint
position targets.

Parameter ``target_d_joint_q``:
    the target relative joint positions in radians.

Parameter ``use_planning``:
    whether or not to use planning. If `true`, the robot will plan the
    trajectory to the target pose with cubic interpolation. If
    `false`, the robot will move directly to the target pose.

Parameter ``time``:
    the time scale to reach the target pose. If planning is disabled,
    this parameter will be ignored.)doc";

static const char *__doc_arm_Robot_add_target_joint_v =
R"doc(Velocity control method. Set the target joint velocities of the robot
arm in joint space. The robot will accelerate / decelerate to target
joint velocities by adding the relative joint velocities to the
current joint velocity targets.

Parameter ``target_joint_v``:
    the target joint velocities in radians per second.)doc";

static const char *__doc_arm_Robot_add_target_relative_rotation =
R"doc(Position control method. Set the target rotation of the robot arm in
Cartesian space. The robot will move to the target pose by adding the
relative rotation to the current planning target. The addition is
performed in **the base frame**.

The reference frame is the base frame, where the origin is located at
the base, x axis is pointing to the front, y axis is pointing to the
left, and z axis is pointing up. The rotation is represented by a
quaternion relative to the `(1, 0, 0)` unit vector.

Parameter ``target_d_rotation``:
    the target relative rotation in quaternion format `( rx, ry, rz,
    rw )`.

Parameter ``use_planning``:
    whether or not to use planning. If `true`, the robot will plan the
    trajectory to the target pose with cubic interpolation. If
    `false`, the robot will move directly to the target pose.

Parameter ``time``:
    the time scale to reach the target pose. If planning is disabled,
    this parameter will be ignored.)doc";

static const char *__doc_arm_Robot_add_target_relative_translation =
R"doc(Set the target relative translation of the robot arm in Cartesian
space. The robot will move to the target pose by adding the relative
translation to the current planning target. The addition is performed
in **the base frame**.

The reference frame is the base frame, where the origin is located at
the base, x axis is pointing to the front, y axis is pointing to the
left, and z axis is pointing up.

Parameter ``target_d_translation``:
    the target relative translation in quaternion format `( rx, ry,
    rz, rw )`.

Parameter ``use_planning``:
    whether or not to use planning. If `true`, the robot will plan the
    trajectory to the target pose with cubic interpolation. If
    `false`, the robot will move directly to the target pose.

Parameter ``time``:
    the time scale to reach the target pose. If planning is disabled,
    this parameter will be ignored.)doc";

static const char *__doc_arm_Robot_add_target_translation =
R"doc(Position control method. Set the target relative translation of the
robot arm in Cartesian space. The robot will move to the target pose
by adding the relative translation to the current planning target.

The reference frame is the base frame, where the origin is located at
the base, x axis is pointing to the front, y axis is pointing to the
left, and z axis is pointing up.

Parameter ``target_d_translation``:
    the target relative translation in meters.

Parameter ``use_planning``:
    whether or not to use planning. If `true`, the robot will plan the
    trajectory to the target pose with cubic interpolation. If
    `false`, the robot will move directly to the target pose.

Parameter ``time``:
    the time scale to reach the target pose. If planning is disabled,
    this parameter will be ignored.)doc";

static const char *__doc_arm_Robot_alter_logging = R"doc()doc";

static const char *__doc_arm_Robot_angle_vel = R"doc()doc";

static const char *__doc_arm_Robot_arm_mode = R"doc()doc";

static const char *__doc_arm_Robot_arm_mode_mutex = R"doc()doc";

static const char *__doc_arm_Robot_base_snap_signal_handle = R"doc()doc";

static const char *__doc_arm_Robot_constrained = R"doc()doc";

static const char *__doc_arm_Robot_current_end = R"doc()doc";

static const char *__doc_arm_Robot_current_joint_err = R"doc()doc";

static const char *__doc_arm_Robot_current_joint_q = R"doc()doc";

static const char *__doc_arm_Robot_current_joint_t = R"doc()doc";

static const char *__doc_arm_Robot_current_joint_temp = R"doc()doc";

static const char *__doc_arm_Robot_current_joint_v = R"doc()doc";

static const char *__doc_arm_Robot_current_mutex = R"doc()doc";

static const char *__doc_arm_Robot_current_pose = R"doc()doc";

static const char *__doc_arm_Robot_current_setting = R"doc()doc";

static const char *__doc_arm_Robot_e2i = R"doc()doc";

static const char *__doc_arm_Robot_end_force = R"doc()doc";

static const char *__doc_arm_Robot_end_mode = R"doc()doc";

static const char *__doc_arm_Robot_end_motor_driver = R"doc()doc";

static const char *__doc_arm_Robot_end_records = R"doc()doc";

static const char *__doc_arm_Robot_end_snap_signal_handle = R"doc()doc";

static const char *__doc_arm_Robot_enter_offline = R"doc()doc";

static const char *__doc_arm_Robot_exit_offline = R"doc()doc";

static const char *__doc_arm_Robot_feedback_joint_q = R"doc()doc";

static const char *__doc_arm_Robot_fk_solver = R"doc()doc";

static const char *__doc_arm_Robot_get_arm_mode = R"doc()doc";

static const char *__doc_arm_Robot_get_base_snap_signal = R"doc()doc";

static const char *__doc_arm_Robot_get_current_end =
R"doc(Get the current end effector position.

The end effector position is a normalized value between 0 and 1, where
0 denotes that the end effector is closed and 1 denotes that the end
effector is open.

Returns:
    double: current end position in meters.)doc";

static const char *__doc_arm_Robot_get_current_joint_error_code = R"doc()doc";

static const char *__doc_arm_Robot_get_current_joint_q =
R"doc(Get the current joint positions in joint space.

Returns:
    vector<double>: current joint positions in radians.)doc";

static const char *__doc_arm_Robot_get_current_joint_t =
R"doc(Get the current joint torques in joint space.

Returns:
    vector<double>: current joint torques in Newton meters.)doc";

static const char *__doc_arm_Robot_get_current_joint_temperature = R"doc()doc";

static const char *__doc_arm_Robot_get_current_joint_v =
R"doc(Get the current joint velocities in joint space.

Returns:
    vector<double>: current joint velocities in radians per second.)doc";

static const char *__doc_arm_Robot_get_current_pose =
R"doc(Get the current end pose in Cartesian space.

The reference frame is the base frame, where the origin is located at
the base, x axis is pointing to the front, y axis is pointing to the
left, and z axis is pointing up. The rotation is represented by a
quaternion relative to the `(1, 0, 0)` unit vector.

Returns:
    vector<vector<double>>: current end pose consisting of translation
    and quaternion rotation in `(( x, y, z ), ( rx, ry, rz, rw ))`
    format.)doc";

static const char *__doc_arm_Robot_get_current_rotation =
R"doc(Get the current rotation part of the end pose.

The rotation is represented by a quaternion relative to the `(1, 0,
0)` unit vector. The reference frame is the base frame, where the
origin is located at the base, x axis is pointing to the front, y axis
is pointing to the left, and z axis is pointing up.

Returns:
    vector<double>: current rotation in quaternion format `( rx, ry,
    rz, rw )`.)doc";

static const char *__doc_arm_Robot_get_current_translation =
R"doc(Get the current translation part of the end pose.

The reference frame is the base frame, where the origin is located at
the base, x axis is pointing to the front, y axis is pointing to the
left, and z axis is pointing up.

Returns:
    vector<double>: current translation in meters.)doc";

static const char *__doc_arm_Robot_get_end_snap_signal = R"doc()doc";

static const char *__doc_arm_Robot_get_motor_response_cnt = R"doc()doc";

static const char *__doc_arm_Robot_get_plan_target_joint_q =
R"doc(Get the planning target joint positions in joint space.

This "target joint position" is the long-term joint position goal of
the robot arm. It is set by the user by calling position controlling
functions with `use_planning` set to `true`.

Returns:
    vector<double>: target joint positions in radians.)doc";

static const char *__doc_arm_Robot_get_plan_target_pose =
R"doc(Get the planning target end pose in Cartesian space.

This "planning target pose" is the long-term end pose goal for
planning of the robot arm. It is set by the user by calling position
controlling functions with `use_planning` set to `true`.

The reference frame is the base frame, where the origin is located at
the base, x axis is pointing to the front, y axis is pointing to the
left, and z axis is pointing up. The rotation is represented by a
quaternion relative to the `(1, 0, 0)` unit vector.

Returns:
    vector<vector<double>>: planning target end pose consisting of
    translation in meters and quaternion rotation in `(( x, y, z ), (
    rx, ry, rz, rw ))` format.)doc";

static const char *__doc_arm_Robot_get_sn =
R"doc(Get the SN code of the robot arm.

Returns:
    std::string: the serial number of the robot arm.)doc";

static const char *__doc_arm_Robot_get_state = R"doc()doc";

static const char *__doc_arm_Robot_get_target_joint_q =
R"doc(Get the target joint positions in joint space.

This "target joint position" is the short-term joint position goal of
the robot arm. It can be either set by the user by calling position
controlling functions with `use_planning` set to `false`, or by the
robot itself when planning is enabled.

Returns:
    vector<double>: target joint positions in radians.)doc";

static const char *__doc_arm_Robot_get_target_joint_t = R"doc()doc";

static const char *__doc_arm_Robot_get_target_joint_v =
R"doc(Get the target joint velocities in joint space.

This "target joint velocity" is the velocity goal of the robot arm. It
is set by the user by calling velocity controlling functions.

Returns:
    vector<double>: target joint velocities in radians per second.)doc";

static const char *__doc_arm_Robot_get_target_pose =
R"doc(Get the target end pose in Cartesian space.

This "target pose" is the short-term end pose goal of the robot arm.
It can be either set by the user by calling position controlling
functions with `use_planning` set to `false`, or by the robot itself
when planning is enabled.

The reference frame is the base frame, where the origin is located at
the base, x axis is pointing to the front, y axis is pointing to the
left, and z axis is pointing up. The rotation is represented by a
quaternion relative to the `(1, 0, 0)` unit vector.

Returns:
    vector<vector<double>>: target end pose consisting of translation
    and quaternion rotation in `(( x, y, z ), ( rx, ry, rz, rw ))`
    format.)doc";

static const char *__doc_arm_Robot_get_target_rotation =
R"doc(Get the rotation part of the target end pose.

The rotation is represented by a quaternion relative to the `(1, 0,
0)` unit vector. The reference frame is the base frame, where the
origin is located at the base, x axis is pointing to the front, y axis
is pointing to the left, and z axis is pointing up.

Returns:
    vector<double>: target rotation in quaternion format `( rx, ry,
    rz, rw )`.)doc";

static const char *__doc_arm_Robot_get_target_translation =
R"doc(Get the translation part of the target end pose.

The reference frame is the base frame, where the origin is located at
the base, x axis is pointing to the front, y axis is pointing to the
left, and z axis is pointing up.

Returns:
    vector<double>: target translation in meters.)doc";

static const char *__doc_arm_Robot_gravity_compensation = R"doc(Enter the gravity compensation mode.)doc";

static const char *__doc_arm_Robot_i2e = R"doc()doc";

static const char *__doc_arm_Robot_id_solver = R"doc()doc";

static const char *__doc_arm_Robot_ignoring_limit = R"doc()doc";

static const char *__doc_arm_Robot_ik_solver = R"doc()doc";

static const char *__doc_arm_Robot_ikv_solver = R"doc()doc";

static const char *__doc_arm_Robot_init_joint_q = R"doc()doc";

static const char *__doc_arm_Robot_interface_board_base = R"doc()doc";

static const char *__doc_arm_Robot_interface_board_end = R"doc()doc";

static const char *__doc_arm_Robot_logger = R"doc()doc";

static const char *__doc_arm_Robot_logging = R"doc()doc";

static const char *__doc_arm_Robot_motor_driver = R"doc()doc";

static const char *__doc_arm_Robot_plan_inference = R"doc()doc";

static const char *__doc_arm_Robot_plan_params = R"doc()doc";

static const char *__doc_arm_Robot_plan_start_timestamp = R"doc()doc";

static const char *__doc_arm_Robot_plan_target_joint_q = R"doc()doc";

static const char *__doc_arm_Robot_plan_target_joint_q_2 = R"doc()doc";

static const char *__doc_arm_Robot_plan_target_pose = R"doc()doc";

static const char *__doc_arm_Robot_plan_total_step = R"doc()doc";

static const char *__doc_arm_Robot_planning = R"doc()doc";

static const char *__doc_arm_Robot_q_records = R"doc()doc";

static const char *__doc_arm_Robot_reached_target_joint_q =
R"doc(Determine if the robot has reached the target joint positions.

Parameter ``threshold``:
    the threshold for determining if the robot has reached the target
    joint positions. The default value is 0.01 radians.

Returns:
    true if the robot has reached the target joint positions, false
    otherwise.)doc";

static const char *__doc_arm_Robot_reached_target_pose =
R"doc(Determine if the robot has reached the target pose.

Parameter ``threshold``:
    the threshold for determining if the robot has reached the target
    pose. The default value is 0.01 meters.

Returns:
    true if the robot has reached the target pose, false otherwise.)doc";

static const char *__doc_arm_Robot_record_load =
R"doc(Load the recorded trajectory from a file.

Parameter ``filepath``:
    the path to the file where the recorded trajectory is saved.)doc";

static const char *__doc_arm_Robot_record_mutex = R"doc()doc";

static const char *__doc_arm_Robot_record_replay = R"doc(Replay the recorded / loaded trajectory of the robot arm.)doc";

static const char *__doc_arm_Robot_record_save =
R"doc(Save the recorded trajectory to a file.

Parameter ``filepath``:
    the path to the file where the recorded trajectory will be saved.)doc";

static const char *__doc_arm_Robot_record_start =
R"doc(Start recording the trajectory of the robot arm.

The robot will record the joint positions, velocities, and torques, as
well as the end position, at each time step. The recorded data can be
saved to a file and replayed later.

Parameter ``record_type``:
    DEPRECATED. This parameter is for backward compatibility and will
    be removed in the future.)doc";

static const char *__doc_arm_Robot_record_stop = R"doc(Stop recording the trajectory of the robot arm.)doc";

static const char *__doc_arm_Robot_record_type = R"doc()doc";

static const char *__doc_arm_Robot_recording = R"doc()doc";

static const char *__doc_arm_Robot_retain_pose = R"doc()doc";

static const char *__doc_arm_Robot_running = R"doc()doc";

static const char *__doc_arm_Robot_set_arm_mode = R"doc()doc";

static const char *__doc_arm_Robot_set_current_joint_q = R"doc()doc";

static const char *__doc_arm_Robot_set_current_joint_t = R"doc()doc";

static const char *__doc_arm_Robot_set_current_joint_v = R"doc()doc";

static const char *__doc_arm_Robot_set_current_pose = R"doc()doc";

static const char *__doc_arm_Robot_set_feedback_joint_q = R"doc()doc";

static const char *__doc_arm_Robot_set_ignore_limit = R"doc()doc";

static const char *__doc_arm_Robot_set_max_current = R"doc()doc";

static const char *__doc_arm_Robot_set_plan_target_pose = R"doc()doc";

static const char *__doc_arm_Robot_set_state = R"doc()doc";

static const char *__doc_arm_Robot_set_target_end = R"doc()doc";

static const char *__doc_arm_Robot_set_target_end_2 =
R"doc(Set the target end position of the robot arm. The end position is a
normalized value between 0 and 1, where 0 denotes that the end
effector is closed and 1 denotes that the end effector is open.

Parameter ``end``:
    the normalized target end position.)doc";

static const char *__doc_arm_Robot_set_target_joint_q = R"doc()doc";

static const char *__doc_arm_Robot_set_target_joint_q_2 =
R"doc(Position control method. Set the target joint positions of the robot
arm in joint space.

Parameter ``target_joint_q``:
    the target joint positions in radians.

Parameter ``use_planning``:
    whether or not to use planning. If `true`, the robot will plan the
    trajectory to the target pose with cubic interpolation. If
    `false`, the robot will move directly to the target pose.

Parameter ``time``:
    the time scale to reach the target pose. If planning is disabled,
    this parameter will be ignored.)doc";

static const char *__doc_arm_Robot_set_target_joint_t = R"doc()doc";

static const char *__doc_arm_Robot_set_target_joint_t_2 =
R"doc(Torque control method. Set the target joint torques of the robot arm
in joint space. @bug This function is not implemented yet.

Parameter ``target_joint_t``:
    the target joint torques in Newton meters.)doc";

static const char *__doc_arm_Robot_set_target_joint_v = R"doc()doc";

static const char *__doc_arm_Robot_set_target_joint_v_2 =
R"doc(Velocity control method. Set the target joint velocities of the robot
arm in joint space.

Parameter ``target_joint_v``:
    the target joint velocities in radians per second.)doc";

static const char *__doc_arm_Robot_set_target_pose = R"doc()doc";

static const char *__doc_arm_Robot_set_target_pose_2 =
R"doc(Position control method. Set the target end pose of the robot arm in
Cartesian space.

The reference frame is the base frame, where the origin is located at
the base, x axis is pointing to the front, y axis is pointing to the
left, and z axis is pointing up. The rotation is represented by a
quaternion relative to the `(1, 0, 0)` unit vector.

Parameter ``target_pose``:
    the target pose in Cartesian space.

Parameter ``use_planning``:
    whether or not to use planning. If `true`, the robot will plan the
    trajectory to the target pose with cubic interpolation. If
    `false`, the robot will move directly to the target pose.

Parameter ``time``:
    the time scale to reach the target pose. If planning is disabled,
    this parameter will be ignored.)doc";

static const char *__doc_arm_Robot_set_target_pose_3 =
R"doc(Position control method. Set the target end pose of the robot arm in
Cartesian space.

The reference frame is the base frame, where the origin is located at
the base, x axis is pointing to the front, y axis is pointing to the
left, and z axis is pointing up. The rotation is represented by a
quaternion relative to the `(1, 0, 0)` unit vector.

Parameter ``target_translation``:
    the target translation in meters.

Parameter ``target_rotation``:
    the target rotation in quaternion format `( rx, ry, rz, rw )`.

Parameter ``use_planning``:
    whether or not to use planning. If `true`, the robot will plan the
    trajectory to the target pose with cubic interpolation. If
    `false`, the robot will move directly to the target pose.

Parameter ``time``:
    the time scale to reach the target pose. If planning is disabled,
    this parameter will be ignored.)doc";

static const char *__doc_arm_Robot_set_target_rotation =
R"doc(Position control method. Set the target rotation of the robot arm in
Cartesian space.

The reference frame is the base frame, where the origin is located at
the base, x axis is pointing to the front, y axis is pointing to the
left, and z axis is pointing up. The rotation is represented by a
quaternion relative to the `(1, 0, 0)` unit vector.

Parameter ``target_rotation``:
    the target rotation in quaternion format `( rx, ry, rz, rw )`.

Parameter ``use_planning``:
    whether or not to use planning. If `true`, the robot will plan the
    trajectory to the target pose with cubic interpolation. If
    `false`, the robot will move directly to the target pose.

Parameter ``time``:
    the time scale to reach the target pose. If planning is disabled,
    this parameter will be ignored.)doc";

static const char *__doc_arm_Robot_set_target_translation =
R"doc(Position control method. Set the target translation of the robot arm
in Cartesian space.

The reference frame is the base frame, where the origin is located at
the base, x axis is pointing to the front, y axis is pointing to the
left, and z axis is pointing up.

Parameter ``target_translation``:
    the target translation in meters.

Parameter ``use_planning``:
    whether or not to use planning. If `true`, the robot will plan the
    trajectory to the target pose with cubic interpolation. If
    `false`, the robot will move directly to the target pose.

Parameter ``time``:
    the time scale to reach the target pose. If planning is disabled,
    this parameter will be ignored.)doc";

static const char *__doc_arm_Robot_set_target_vel =
R"doc(Velocity control method. Set the target end velocities of the robot
arm in Cartesian space.

@bug This function is not implemented yet.

Parameter ``target_vel``:
    the target end velocities in meters per second.)doc";

static const char *__doc_arm_Robot_set_zero =
R"doc(Set current position of the motors as the zero position This function
will set the current position of the motors as the zero position. This
function should be **NOT** be called while the robot is moving.)doc";

static const char *__doc_arm_Robot_snap_update = R"doc()doc";

static const char *__doc_arm_Robot_state = R"doc()doc";

static const char *__doc_arm_Robot_state_mutex = R"doc()doc";

static const char *__doc_arm_Robot_state_record = R"doc()doc";

static const char *__doc_arm_Robot_stop_gravity_compensation = R"doc(Exit the gravity compensation mode.)doc";

static const char *__doc_arm_Robot_sync_pose = R"doc()doc";

static const char *__doc_arm_Robot_t_records = R"doc()doc";

static const char *__doc_arm_Robot_target_end = R"doc()doc";

static const char *__doc_arm_Robot_target_joint_q = R"doc()doc";

static const char *__doc_arm_Robot_target_joint_t = R"doc()doc";

static const char *__doc_arm_Robot_target_joint_v = R"doc()doc";

static const char *__doc_arm_Robot_target_mutex = R"doc()doc";

static const char *__doc_arm_Robot_target_pose = R"doc()doc";

static const char *__doc_arm_Robot_thread_log = R"doc()doc";

static const char *__doc_arm_Robot_thread_plan = R"doc()doc";

static const char *__doc_arm_Robot_thread_record = R"doc()doc";

static const char *__doc_arm_Robot_thread_snap = R"doc()doc";

static const char *__doc_arm_Robot_thread_sync_pose = R"doc()doc";

static const char *__doc_arm_Robot_thread_update_motor = R"doc()doc";

static const char *__doc_arm_Robot_time_records = R"doc()doc";

static const char *__doc_arm_Robot_update_motor = R"doc()doc";

static const char *__doc_arm_Robot_update_mutex = R"doc()doc";

static const char *__doc_arm_Robot_v_records = R"doc()doc";

static const char *__doc_arm_Robot_valid_joint_q = R"doc()doc";

static const char *__doc_arm_Robot_valid_joint_q_2 =
R"doc(Determine if the given target joint positions are valid for AIRBOT
Play.

Parameter ``joint_q``:
    the target joint positions in radians.

Returns:
    true if the robot arm is in the target joint positions, false
    otherwise.)doc";

static const char *__doc_arm_Robot_valid_target_pose =
R"doc(Determine if the given target pose is valid for AIRBOT Play.

Parameter ``target_pose``:
    the target pose in Cartesian space.

Returns:
    true if the pose is valid, false otherwise.)doc";

static const char *__doc_arm_Robot_with_feedback = R"doc()doc";

static const char *__doc_arm_Robot_wp_records = R"doc()doc";

static const char *__doc_arm_calc = R"doc()doc";

static const char *__doc_arm_calc_plan = R"doc()doc";

static const char *__doc_arm_max_diff = R"doc()doc";

static const char *__doc_arm_plan_infer = R"doc()doc";

static const char *__doc_arm_vector_difference = R"doc()doc";

static const char *__doc_get_microsecond_now = R"doc()doc";

static const char *__doc_get_millisecond_now = R"doc()doc";

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif


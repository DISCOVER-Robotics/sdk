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


static const char *__doc_KibanaLogger = R"doc()doc";

static const char *__doc_KibanaLogger_KibanaLogger = R"doc()doc";

static const char *__doc_KibanaLogger_client = R"doc()doc";

static const char *__doc_KibanaLogger_inited = R"doc()doc";

static const char *__doc_KibanaLogger_push_remote_log_once = R"doc()doc";

static const char *__doc_KibanaLogger_topic_name = R"doc()doc";

static const char *__doc_KibanaLogger_url = R"doc()doc";

static const char *__doc_Robot = R"doc()doc";

static const char *__doc_arm_Robot = R"doc()doc";

static const char *__doc_arm_Robot_Robot = R"doc()doc";

static const char *__doc_arm_Robot_add_target_joint_q = R"doc()doc";

static const char *__doc_arm_Robot_add_target_joint_v = R"doc()doc";

static const char *__doc_arm_Robot_add_target_relative_rotation = R"doc()doc";

static const char *__doc_arm_Robot_add_target_relative_translation = R"doc()doc";

static const char *__doc_arm_Robot_add_target_translation = R"doc()doc";

static const char *__doc_arm_Robot_arm_mode = R"doc(Robot status)doc";

static const char *__doc_arm_Robot_change_mode = R"doc()doc";

static const char *__doc_arm_Robot_check_mode_change = R"doc()doc";

static const char *__doc_arm_Robot_cmd_mutex = R"doc(Mutexes)doc";

static const char *__doc_arm_Robot_counter = R"doc()doc";

static const char *__doc_arm_Robot_e2i = R"doc()doc";

static const char *__doc_arm_Robot_end_effector_type = R"doc(Robot parameters)doc";

static const char *__doc_arm_Robot_end_motor_driver = R"doc()doc";

static const char *__doc_arm_Robot_fb_mutex = R"doc()doc";

static const char *__doc_arm_Robot_fk_solver = R"doc(Robot modules)doc";

static const char *__doc_arm_Robot_get_current_end = R"doc()doc";

static const char *__doc_arm_Robot_get_current_joint_error_code = R"doc()doc";

static const char *__doc_arm_Robot_get_current_joint_q = R"doc()doc";

static const char *__doc_arm_Robot_get_current_joint_t = R"doc()doc";

static const char *__doc_arm_Robot_get_current_joint_temperature = R"doc()doc";

static const char *__doc_arm_Robot_get_current_joint_v = R"doc()doc";

static const char *__doc_arm_Robot_get_current_pose = R"doc()doc";

static const char *__doc_arm_Robot_get_current_rotation = R"doc()doc";

static const char *__doc_arm_Robot_get_current_translation = R"doc()doc";

static const char *__doc_arm_Robot_get_sn = R"doc()doc";

static const char *__doc_arm_Robot_i2e = R"doc()doc";

static const char *__doc_arm_Robot_id_solver = R"doc()doc";

static const char *__doc_arm_Robot_ik_solver = R"doc()doc";

static const char *__doc_arm_Robot_ikv_solver = R"doc()doc";

static const char *__doc_arm_Robot_interface_board_base = R"doc()doc";

static const char *__doc_arm_Robot_interface_board_end = R"doc()doc";

static const char *__doc_arm_Robot_is_init = R"doc()doc";

static const char *__doc_arm_Robot_is_running = R"doc()doc";

static const char *__doc_arm_Robot_joint_safe_detect = R"doc()doc";

static const char *__doc_arm_Robot_joint_vel_limit = R"doc()doc";

static const char *__doc_arm_Robot_kibana_logger = R"doc()doc";

static const char *__doc_arm_Robot_last_logging_time = R"doc()doc";

static const char *__doc_arm_Robot_last_update_time = R"doc()doc";

static const char *__doc_arm_Robot_logger = R"doc(Logging)doc";

static const char *__doc_arm_Robot_logging = R"doc()doc";

static const char *__doc_arm_Robot_logging_freq = R"doc()doc";

static const char *__doc_arm_Robot_logging_queue = R"doc()doc";

static const char *__doc_arm_Robot_logging_thread = R"doc()doc";

static const char *__doc_arm_Robot_main_update_thread = R"doc(Thread handles)doc";

static const char *__doc_arm_Robot_manual_mode = R"doc(test)doc";

static const char *__doc_arm_Robot_motor_driver = R"doc()doc";

static const char *__doc_arm_Robot_offline_mode = R"doc()doc";

static const char *__doc_arm_Robot_on_mode_change = R"doc()doc";

static const char *__doc_arm_Robot_online_mode = R"doc()doc";

static const char *__doc_arm_Robot_plan_target_joint_q = R"doc()doc";

static const char *__doc_arm_Robot_record_load = R"doc()doc";

static const char *__doc_arm_Robot_record_mutex = R"doc()doc";

static const char *__doc_arm_Robot_record_save = R"doc()doc";

static const char *__doc_arm_Robot_record_start = R"doc()doc";

static const char *__doc_arm_Robot_record_stop = R"doc()doc";

static const char *__doc_arm_Robot_recorded_data = R"doc()doc";

static const char *__doc_arm_Robot_replay_data = R"doc()doc";

static const char *__doc_arm_Robot_replay_index = R"doc()doc";

static const char *__doc_arm_Robot_replay_inplace_time = R"doc()doc";

static const char *__doc_arm_Robot_replay_start = R"doc()doc";

static const char *__doc_arm_Robot_reported_time = R"doc()doc";

static const char *__doc_arm_Robot_reset_error = R"doc()doc";

static const char *__doc_arm_Robot_robot_cmd_data = R"doc()doc";

static const char *__doc_arm_Robot_robot_fb_data = R"doc()doc";

static const char *__doc_arm_Robot_robot_plan_data = R"doc()doc";

static const char *__doc_arm_Robot_safe_joint_q = R"doc()doc";

static const char *__doc_arm_Robot_set_frame = R"doc()doc";

static const char *__doc_arm_Robot_set_max_current = R"doc()doc";

static const char *__doc_arm_Robot_set_state = R"doc()doc";

static const char *__doc_arm_Robot_set_target_end = R"doc()doc";

static const char *__doc_arm_Robot_set_target_joint_mit = R"doc()doc";

static const char *__doc_arm_Robot_set_target_joint_mit_2 = R"doc()doc";

static const char *__doc_arm_Robot_set_target_joint_q = R"doc()doc";

static const char *__doc_arm_Robot_set_target_joint_v = R"doc()doc";

static const char *__doc_arm_Robot_set_target_pose = R"doc()doc";

static const char *__doc_arm_Robot_set_target_pose_2 = R"doc()doc";

static const char *__doc_arm_Robot_set_target_rotation = R"doc()doc";

static const char *__doc_arm_Robot_set_target_translation = R"doc()doc";

static const char *__doc_arm_Robot_set_target_vel = R"doc()doc";

static const char *__doc_arm_Robot_slow_joint_q = R"doc()doc";

static const char *__doc_arm_Robot_update_once = R"doc()doc";

static const char *__doc_arm_Robot_use_planning = R"doc()doc";

static const char *__doc_arm_Robot_valid_joint_q = R"doc()doc";

static const char *__doc_arm_Robot_valid_joint_v = R"doc()doc";

static const char *__doc_arm_Robot_valid_target_pose = R"doc()doc";

static const char *__doc_arm_Robot_write_cmd_data = R"doc()doc";

static const char *__doc_arm_Robot_write_fb_data = R"doc()doc";

static const char *__doc_arm_calc = R"doc()doc";

static const char *__doc_arm_calc_plan = R"doc()doc";

static const char *__doc_arm_plan_infer = R"doc()doc";

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif


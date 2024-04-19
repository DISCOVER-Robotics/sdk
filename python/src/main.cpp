#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <airbot/airbot.hpp>
#include <airbot/modules/tools/logger/log.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#ifdef DOCSTRINGS_EXISTS
#include "docstrings.h"
#else
#define DOC(...) R"doc()doc"
#endif

namespace py = pybind11;
using namespace arm;

std::unique_ptr<Robot> createAgent(std::string urdf_path = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf",
                                   std::string direction = "down", std::string can_interface = "can0",
                                   double vel = M_PI, std::string end_mode = "newteacher", bool constraint = false) {
  return std::make_unique<Robot>(urdf_path, can_interface, direction, vel, end_mode, constraint);
}

std::unique_ptr<MotorDriver> createMotor(uint16_t id, const char *interface, std::string type) {
  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      (std::string("logs/airbot_play-") + get_timestring() + ".log").c_str(), 1024 * 1024, 10, false));
  auto logger_ = arm::setup_logger(sinks);
  spdlog::flush_every(std::chrono::seconds(1));
  logger_->set_level(spdlog::level::info);
  return MotorDriver::MotorCreate(id, interface, logger_, type);
}

PYBIND11_MODULE(airbot, m) {
  m.doc() = "airbot";
  m.attr("__version__") = AIRBOT_VERSION;
  m.attr("AIRBOT_PLAY_URDF") = URDF_INSTALL_PATH + "airbot_play_v2_1.urdf";
  m.attr("AIRBOT_PLAY_WITH_GRIPPER_URDF") = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf";

  py::class_<Robot, std::unique_ptr<Robot>>(m, "Robot")
      .def("alter_logging", &Robot::alter_logging, DOC(arm, Robot, alter_logging))
      .def("get_target_pose", &Robot::get_target_pose, DOC(arm, Robot, get_target_pose))
      .def("get_target_joint_q", &Robot::get_target_joint_q, DOC(arm, Robot, get_target_joint_q))
      .def("get_target_joint_v", &Robot::get_target_joint_v, DOC(arm, Robot, get_target_joint_v))
      .def("get_target_joint_t", &Robot::get_target_joint_t, DOC(arm, Robot, get_target_joint_t))
      .def("get_target_translation", &Robot::get_target_translation, DOC(arm, Robot, get_target_translation))
      .def("get_target_rotation", &Robot::get_target_rotation, DOC(arm, Robot, get_target_rotation))
      .def("get_current_pose", &Robot::get_current_pose, DOC(arm, Robot, get_current_pose))
      .def("get_current_joint_q", &Robot::get_current_joint_q, DOC(arm, Robot, get_current_joint_q))
      .def("get_current_joint_v", &Robot::get_current_joint_v, DOC(arm, Robot, get_current_joint_v))
      .def("get_current_joint_t", &Robot::get_current_joint_t, DOC(arm, Robot, get_current_joint_t))
      .def("get_current_translation", &Robot::get_current_translation, DOC(arm, Robot, get_current_translation))
      .def("get_current_rotation", &Robot::get_current_rotation, DOC(arm, Robot, get_current_rotation))
      .def("get_current_end", &Robot::get_current_end, DOC(arm, Robot, get_current_end))
      .def("get_current_joint_error_code", &Robot::get_current_joint_error_code,
           DOC(arm, Robot, get_current_joint_error_code))
      .def("get_current_joint_temperature", &Robot::get_current_joint_temperature,
           DOC(arm, Robot, get_current_joint_temperature))
      .def("get_motor_response_cnt", &Robot::get_motor_response_cnt, DOC(arm, Robot, get_motor_response_cnt))
      .def("get_sn", &Robot::get_sn, DOC(arm, Robot, get_sn))
      .def("set_target_pose",
           py::overload_cast<const std::vector<std::vector<double>> &, bool, double>(&Robot::set_target_pose),
           py::arg("target_pose"), py::arg("use_planning") = true, py::arg("time") = 1.,
           DOC(arm, Robot, set_target_pose))
      .def("set_target_pose",
           py::overload_cast<const std::vector<double> &, const std::vector<double> &, bool, double>(
               &Robot::set_target_pose),
           py::arg("target_translation"), py::arg("target_rotation"), py::arg("use_planning") = true,
           py::arg("time") = 1., DOC(arm, Robot, set_target_pose))
      .def("set_target_translation", &Robot::set_target_translation, py::arg("target_translation"),
           py::arg("use_planning") = true, py::arg("time") = 1., DOC(arm, Robot, set_target_translation))
      .def("add_target_translation", &Robot::add_target_translation, py::arg("target_d_translation"),
           py::arg("use_planning") = true, py::arg("time") = 1., DOC(arm, Robot, add_target_translation))
      .def("add_target_relative_translation", &Robot::add_target_relative_translation, py::arg("target_d_translation"),
           py::arg("use_planning") = true, py::arg("time") = 1., DOC(arm, Robot, add_target_relative_translation))
      .def("set_target_rotation", &Robot::set_target_rotation, py::arg("target_rotation"),
           py::arg("use_planning") = true, py::arg("time") = 1., DOC(arm, Robot, set_target_rotation))
      .def("add_target_relative_rotation", &Robot::add_target_relative_rotation, py::arg("target_d_rotation"),
           py::arg("use_planning") = true, py::arg("time") = 1., DOC(arm, Robot, add_target_relative_rotation))
      .def("set_target_vel", &Robot::set_target_vel, py::arg("target_vel"), DOC(arm, Robot, set_target_vel))
      .def("set_target_joint_q", &Robot::set_target_joint_q, py::arg("target_joint_q"), py::arg("use_planning") = true,
           py::arg("time") = 1., DOC(arm, Robot, set_target_joint_q))
      .def("add_target_joint_q", &Robot::add_target_joint_q, py::arg("target_d_joint_q"),
           py::arg("use_planning") = true, py::arg("time") = 1., DOC(arm, Robot, add_target_joint_q))
      .def("set_target_joint_v", &Robot::set_target_joint_v, py::arg("target_joint_v"),
           DOC(arm, Robot, set_target_joint_v))
      .def("add_target_joint_v", &Robot::add_target_joint_v, py::arg("target_d_joint_v"),
           DOC(arm, Robot, add_target_joint_v))
      .def("set_target_joint_t", &Robot::set_target_joint_t, py::arg("target_joint_t"),
           DOC(arm, Robot, set_target_joint_t))
      .def("set_target_end", &Robot::set_target_end, py::arg("target_end"), DOC(arm, Robot, set_target_end))
      .def("record_start", &Robot::record_start, DOC(arm, Robot, record_start))
      .def("record_stop", &Robot::record_stop, DOC(arm, Robot, record_stop))
      .def("record_replay", &Robot::record_replay, DOC(arm, Robot, record_replay))
      .def("record_save", &Robot::record_save, py::arg("filepath"), DOC(arm, Robot, record_save))
      .def("record_load", &Robot::record_load, py::arg("filepath"), DOC(arm, Robot, record_load))
      .def("gravity_compensation", &Robot::gravity_compensation, DOC(arm, Robot, gravity_compensation))
      .def("stop_gravity_compensation", &Robot::stop_gravity_compensation, DOC(arm, Robot, stop_gravity_compensation))
      .def("set_max_current", &Robot::set_max_current, py::arg("max"), py::arg("second") = 1.,
           DOC(arm, Robot, set_max_current));

  py::class_<MotorDriver, std::unique_ptr<MotorDriver>>(m, "MotorDriver")
      .def("MotorLock", &MotorDriver::MotorLock, DOC(arm, MotorDriver, MotorLock))
      .def("MotorUnlock", &MotorDriver::MotorUnlock, DOC(arm, MotorDriver, MotorUnlock))
      .def("MotorInit", &MotorDriver::MotorInit, DOC(arm, MotorDriver, MotorInit))
      .def("MotorDeInit", &MotorDriver::MotorDeInit, DOC(arm, MotorDriver, MotorDeInit))
      .def("MotorSetZero", &MotorDriver::MotorSetZero, DOC(arm, MotorDriver, MotorSetZero))
      .def("MotorWriteFlash", &MotorDriver::MotorWriteFlash, DOC(arm, MotorDriver, MotorWriteFlash))
      .def("MotorBoundary", &MotorDriver::MotorBoundary, DOC(arm, MotorDriver, MotorBoundary))
      .def("MotorGetParam", &MotorDriver::MotorGetParam, py::arg("target_joint_t"),
           DOC(arm, MotorDriver, MotorGetParam))
      .def("MotorPosModeCmd", &MotorDriver::MotorPosModeCmd, py::arg("pos"), py::arg("spd"),
           py::arg("ignore_limit") = false, DOC(arm, MotorDriver, MotorPosModeCmd))

      .def("MotorSpdModeCmd", &MotorDriver::MotorSpdModeCmd, py::arg("spd"), DOC(arm, MotorDriver, MotorSpdModeCmd))
      .def("MotorMitModeCmd", &MotorDriver::MotorMitModeCmd, py::arg("f_p"), py::arg("f_v"), py::arg("f_kp"),
           py::arg("f_kd"), py::arg("f_t"), DOC(arm, MotorDriver, MotorMitModeCmd))
      .def("MotorSetPosParam", &MotorDriver::MotorSetPosParam, py::arg("kp"), py::arg("kd"),
           DOC(arm, MotorDriver, MotorSetPosParam))
      .def("MotorSetSpdParam", &MotorDriver::MotorSetSpdParam, py::arg("kp"), py::arg("ki"),
           DOC(arm, MotorDriver, MotorSetSpdParam))
      .def("MotorSetFilterParam", &MotorDriver::MotorSetFilterParam, py::arg("position_kd_filter"), py::arg("kd_spd"),
           DOC(arm, MotorDriver, MotorSetFilterParam))
      .def("set_motor_id", &MotorDriver::set_motor_id, py::arg("motor_id"), DOC(arm, MotorDriver, set_motor_id))
      .def("set_motor_control_mode", &MotorDriver::set_motor_control_mode, py::arg("motor_control_mode"),
           DOC(arm, MotorDriver, set_motor_control_mode))
      .def("get_response_count", &MotorDriver::get_response_count, DOC(arm, MotorDriver, get_response_count))
      .def("MotorResetID", &MotorDriver::MotorResetID, DOC(arm, MotorDriver, MotorResetID))
      .def("MotorErrorModeCmd", &MotorDriver::MotorErrorModeCmd, DOC(arm, MotorDriver, MotorErrorModeCmd))
      .def("MotorCurrentDetect", &MotorDriver::MotorCurrentDetect, DOC(arm, MotorDriver, MotorCurrentDetect))
      .def("MotorCommunicationDetect", &MotorDriver::MotorCommunicationDetect,
           DOC(arm, MotorDriver, MotorCommunicationDetect))
      .def("MotorTemperatureDetect", &MotorDriver::MotorTemperatureDetect,
           DOC(arm, MotorDriver, MotorTemperatureDetect))
      .def("MotorErrorDetect", &MotorDriver::MotorErrorDetect, DOC(arm, MotorDriver, MotorErrorDetect))
      .def("MotorErrorModeCmd", &MotorDriver::MotorErrorModeCmd, DOC(arm, MotorDriver, MotorErrorModeCmd))
      .def("get_motor_id", &MotorDriver::get_motor_id, DOC(arm, MotorDriver, get_motor_id))
      .def("get_motor_control_mode", &MotorDriver::get_motor_control_mode,
           DOC(arm, MotorDriver, get_motor_control_mode))
      .def("get_error_id", &MotorDriver::get_error_id, DOC(arm, MotorDriver, get_error_id))
      .def("get_timeout", &MotorDriver::get_timeout, DOC(arm, MotorDriver, get_timeout))
      .def("get_gear_ratio", &MotorDriver::get_gear_ratio, DOC(arm, MotorDriver, get_gear_ratio))
      .def("get_write_para_res", &MotorDriver::get_write_para_res, DOC(arm, MotorDriver, get_write_para_res))
      .def("get_motor_pos", &MotorDriver::get_motor_pos, DOC(arm, MotorDriver, get_motor_pos))
      .def("get_motor_spd", &MotorDriver::get_motor_spd, DOC(arm, MotorDriver, get_motor_spd))
      .def("get_motor_current", &MotorDriver::get_motor_current, DOC(arm, MotorDriver, get_motor_current))

      .def("get_motor_error_id", &MotorDriver::get_motor_error_id, DOC(arm, MotorDriver, get_motor_error_id))
      .def("get_motor_temperature", &MotorDriver::get_motor_temperature, DOC(arm, MotorDriver, get_motor_temperature))
      .def("get_motor_acceleration", &MotorDriver::get_motor_acceleration,
           DOC(arm, MotorDriver, get_motor_acceleration))
      .def("get_motor_kp_spd", &MotorDriver::get_motor_kp_spd, DOC(arm, MotorDriver, get_motor_kp_spd))
      .def("get_motor_ki_spd", &MotorDriver::get_motor_ki_spd, DOC(arm, MotorDriver, get_motor_ki_spd))
      .def("get_motor_kd_spd", &MotorDriver::get_motor_kd_spd, DOC(arm, MotorDriver, get_motor_kd_spd))
      .def("get_motor_kp_pos", &MotorDriver::get_motor_kp_pos, DOC(arm, MotorDriver, get_motor_kp_pos))
      .def("get_motor_ki_pos", &MotorDriver::get_motor_ki_pos, DOC(arm, MotorDriver, get_motor_ki_pos))
      .def("get_motor_kd_pos", &MotorDriver::get_motor_kd_pos, DOC(arm, MotorDriver, get_motor_kd_pos))
      .def("set_max_current", &MotorDriver::set_max_current, DOC(arm, MotorDriver, set_max_current))
      .def("get_max_current", &MotorDriver::get_max_current, DOC(arm, MotorDriver, get_max_current))
      .def("get_write_para_res_and_clear", &MotorDriver::get_write_para_res_and_clear,
           DOC(arm, MotorDriver, get_write_para_res_and_clear));

  m.def("create_agent", &createAgent, py::arg("urdf_path") = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf",
        py::arg("direction") = "down", py::arg("can_interface") = "can0", py::arg("vel") = M_PI,
        py::arg("end_mode") = "newteacher", py::arg("constraint") = false, DOC(arm, Robot, Robot));
  m.def("create_motor", &createMotor, py::arg("motor_id"), py::arg("interface"), py::arg("type"));
};

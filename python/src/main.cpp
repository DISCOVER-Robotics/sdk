#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <airbot/airbot.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "doc.hpp"

namespace py = pybind11;

std::unique_ptr<arm::Robot<6>> createAgent(std::string urdf_path = URDF_INSTALL_PATH +
                                                                   "airbot_play_v2_1_with_gripper.urdf",
                                           std::string direction = "down", std::string can_interface = "can0",
                                           double vel = 0.2, std::string end_mode = "newteacher",
                                           std::string forearm_type = "DM") {
  return std::make_unique<arm::Robot<6>>(urdf_path, can_interface, direction, vel, end_mode, forearm_type);
}

std::unique_ptr<MotorDriver> createMotor(uint16_t id, const char *interface, std::string type) {
  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      (std::string("logs/airbot_play-") + get_timestring() + ".log").c_str(), 1024 * 1024, 10, false));
  auto logger_ = setup_logger(sinks);
  spdlog::flush_every(std::chrono::seconds(1));
  logger_->set_level(spdlog::level::info);
  return MotorDriver::MotorCreate(id, interface, type);
}
PYBIND11_MODULE(airbot, m) {
  m.doc() = "Doc for airbot module";
  m.attr("__version__") = AIRBOT_VERSION;
  m.attr("AIRBOT_PLAY_URDF") = URDF_INSTALL_PATH + "airbot_play_v2_1.urdf"; /*Test*/
  m.attr("AIRBOT_PLAY_WITH_GRIPPER_URDF") = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf";
  m.attr("AIRBOT_PLAY_JOINT_LOWER_LIMITS") = std::array<double, 6>{
      MotorDriver::joint_lower_bounder_[0] + 1 * M_PI / 180, MotorDriver::joint_lower_bounder_[1] + 1 * M_PI / 180,
      MotorDriver::joint_lower_bounder_[2] + 1 * M_PI / 180, MotorDriver::joint_lower_bounder_[3] + 1 * M_PI / 180,
      MotorDriver::joint_lower_bounder_[4] + 1 * M_PI / 180, MotorDriver::joint_lower_bounder_[5] + 1 * M_PI / 180};
  m.attr("AIRBOT_PLAY_JOINT_UPPER_LIMITS") = std::array<double, 6>{
      MotorDriver::joint_upper_bounder_[0] - 1 * M_PI / 180, MotorDriver::joint_upper_bounder_[1] - 1 * M_PI / 180,
      MotorDriver::joint_upper_bounder_[2] - 1 * M_PI / 180, MotorDriver::joint_upper_bounder_[3] - 1 * M_PI / 180,
      MotorDriver::joint_upper_bounder_[4] - 1 * M_PI / 180, MotorDriver::joint_upper_bounder_[5] - 1 * M_PI / 180};

  py::class_<arm::Robot<6>, std::unique_ptr<arm::Robot<6>>>(m, "Robot", DOC(arm, Robot))
      .def("logging", &arm::Robot<6>::logging, py::arg("logging"), DOC(arm, Robot, logging))
      .def("get_current_pose", &arm::Robot<6>::get_current_pose, DOC(arm, Robot, get_current_pose))
      .def("get_current_joint_q", &arm::Robot<6>::get_current_joint_q, DOC(arm, Robot, get_current_joint_q))
      .def("get_current_joint_v", &arm::Robot<6>::get_current_joint_v, DOC(arm, Robot, get_current_joint_v))
      .def("get_current_joint_t", &arm::Robot<6>::get_current_joint_t, DOC(arm, Robot, get_current_joint_t))
      .def("get_current_translation", &arm::Robot<6>::get_current_translation, DOC(arm, Robot, get_current_translation))
      .def("get_current_rotation", &arm::Robot<6>::get_current_rotation, DOC(arm, Robot, get_current_rotation))
      .def("get_current_end", &arm::Robot<6>::get_current_end, DOC(arm, Robot, get_current_end))
      .def("get_current_joint_error_code", &arm::Robot<6>::get_current_joint_error_code,
           DOC(arm, Robot, get_current_joint_error_code))
      .def("get_current_joint_temperature", &arm::Robot<6>::get_current_joint_temperature,
           DOC(arm, Robot, get_current_joint_temperature))
      //  .def("get_motor_response_cnt", &arm::Robot<6>::get_motor_response_cnt, DOC(arm, Robot,
      //  get_motor_response_cnt))
      .def("get_sn", &arm::Robot<6>::get_sn, DOC(arm, Robot, get_sn))

      .def("set_target_pose", py::overload_cast<const Frame &, bool, double, bool>(&arm::Robot<6>::set_target_pose),
           py::arg("target_pose"), py::arg("use_planning") = true, py::arg("vel") = 0.2, py::arg("blocking") = false,
           DOC(arm, Robot, set_target_pose))
      .def(
          "set_target_pose",
          py::overload_cast<const Translation &, const Rotation &, bool, double, bool>(&arm::Robot<6>::set_target_pose),
          py::arg("target_translation"), py::arg("target_rotation"), py::arg("use_planning") = true,
          py::arg("vel") = 0.2, py::arg("blocking") = false, DOC(arm, Robot, set_target_pose))
      .def("set_target_translation", &arm::Robot<6>::set_target_translation, py::arg("target_translation"),
           py::arg("use_planning") = true, py::arg("vel") = 0.2, py::arg("blocking") = false,
           DOC(arm, Robot, set_target_translation))
      .def("add_target_translation", &arm::Robot<6>::add_target_translation, py::arg("target_d_translation"),
           py::arg("use_planning") = true, py::arg("vel") = 0.2, py::arg("blocking") = false,
           DOC(arm, Robot, add_target_translation))
      .def("add_target_relative_translation", &arm::Robot<6>::add_target_relative_translation,
           py::arg("target_d_translation"), py::arg("use_planning") = true, py::arg("vel") = 0.2,
           py::arg("blocking") = false, DOC(arm, Robot, add_target_relative_translation))
      .def("set_target_rotation", &arm::Robot<6>::set_target_rotation, py::arg("target_rotation"),
           py::arg("use_planning") = true, py::arg("vel") = 0.2, py::arg("blocking") = false,
           DOC(arm, Robot, set_target_rotation))
      .def("add_target_relative_rotation", &arm::Robot<6>::add_target_relative_rotation, py::arg("target_d_rotation"),
           py::arg("use_planning") = true, py::arg("vel") = 0.2, py::arg("blocking") = false,
           DOC(arm, Robot, add_target_relative_rotation))
      .def("set_target_vel", &arm::Robot<6>::set_target_vel, py::arg("target_vel"), DOC(arm, Robot, set_target_vel))

      .def("set_target_joint_q", &arm::Robot<6>::set_target_joint_q, py::arg("target_joint_q"),
           py::arg("use_planning") = true, py::arg("vel") = 0.2, py::arg("blocking") = false,
           DOC(arm, Robot, set_target_joint_q))
      .def("add_target_joint_q", &arm::Robot<6>::add_target_joint_q, py::arg("target_d_joint_q"),
           py::arg("use_planning") = true, py::arg("vel") = 0.2, py::arg("blocking") = false,
           DOC(arm, Robot, add_target_joint_q))
      .def("set_target_joint_v", &arm::Robot<6>::set_target_joint_v, py::arg("target_joint_v"),
           DOC(arm, Robot, set_target_joint_v))
      .def("add_target_joint_v", &arm::Robot<6>::add_target_joint_v, py::arg("target_d_joint_v"),
           DOC(arm, Robot, add_target_joint_v))
      .def("set_target_joint_mit",
           py::overload_cast<const Joints<6> &, const Joints<6> &, const Joints<6> &, const Joints<6> &>(
               &arm::Robot<6>::set_target_joint_mit),
           py::arg("target_joint_q"), py::arg("target_joint_v"), py::arg("target_joint_kp"), py::arg("target_joint_kd"),
           DOC(arm, Robot, set_target_joint_mit))
      .def("set_target_joint_mit",
           py::overload_cast<const Joints<6> &, const Joints<6> &, const Joints<6> &, const Joints<6> &,
                             const Joints<6> &>(&arm::Robot<6>::set_target_joint_mit),
           py::arg("target_joint_q"), py::arg("target_joint_v"), py::arg("target_joint_kp"), py::arg("target_joint_kd"),
           py::arg("target_joint_t"), DOC(arm, Robot, set_target_joint_mit))
      .def("set_target_end", &arm::Robot<6>::set_target_end, py::arg("target_end"), py::arg("blocking") = false,
           DOC(arm, Robot, set_target_end))

      .def("valid_target_pose", &arm::Robot<6>::valid_target_pose, py::arg("target_pose"),
           DOC(arm, Robot, valid_target_pose))
      .def("valid_joint_q", &arm::Robot<6>::valid_joint_q, py::arg("joint_q"), DOC(arm, Robot, valid_joint_q))
      .def("safe_joint_q", &arm::Robot<6>::safe_joint_q, py::arg("joint_q"), DOC(arm, Robot, safe_joint_q))

      .def("record_save", &arm::Robot<6>::record_save, py::arg("filepath"), DOC(arm, Robot, record_save))
      .def("record_load", &arm::Robot<6>::record_load, py::arg("filepath"), DOC(arm, Robot, record_load))
      .def("record_start", &arm::Robot<6>::record_start, DOC(arm, Robot, record_start))
      .def("record_stop", &arm::Robot<6>::record_stop, DOC(arm, Robot, record_stop))

      .def("replay_start", &arm::Robot<6>::replay_start, DOC(arm, Robot, replay_start))

      .def("manual_mode", &arm::Robot<6>::manual_mode, DOC(arm, Robot, manual_mode))
      .def("offline_mode", &arm::Robot<6>::offline_mode, DOC(arm, Robot, offline_mode))
      .def("online_mode", &arm::Robot<6>::online_mode, DOC(arm, Robot, online_mode))
      .def("reset_error", &arm::Robot<6>::reset_error, DOC(arm, Robot, reset_error))
      .def("set_max_current", &arm::Robot<6>::set_max_current, py::arg("max"), DOC(arm, Robot, set_max_current));

  m.def("create_agent", &createAgent, py::arg("urdf_path") = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf",
        py::arg("direction") = "down", py::arg("can_interface") = "can0", py::arg("vel") = 0.2,
        py::arg("end_mode") = "newteacher", py::arg("forearm_type") = "DM");
};

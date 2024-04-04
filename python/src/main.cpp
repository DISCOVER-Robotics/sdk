#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <airbot/airbot.hpp>
#include <memory>

namespace py = pybind11;
using namespace arm;

std::unique_ptr<Robot> createAgent(std::string urdf_path = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf",
                                   std::string direction = "down", std::string can_interface = "can0",
                                   double vel = M_PI, std::string end_mode = "newteacher", bool constraint = false) {
  return std::make_unique<Robot>(urdf_path, can_interface, direction, vel, end_mode, constraint);
}

PYBIND11_MODULE(airbot, m) {
  m.doc() = "airbot";
  m.attr("__version__") = AIRBOT_VERSION;
  m.attr("AIRBOT_PLAY_URDF") = URDF_INSTALL_PATH + "airbot_play_v2_1.urdf";
  m.attr("AIRBOT_PLAY_WITH_GRIPPER_URDF") = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf";

  py::class_<Robot, std::unique_ptr<Robot>>(m, "Robot")
      .def("alter_logging", &Robot::alter_logging)
      .def("get_target_pose", &Robot::get_target_pose)
      .def("get_target_joint_q", &Robot::get_target_joint_q)
      .def("get_target_joint_v", &Robot::get_target_joint_v)
      .def("get_target_joint_t", &Robot::get_target_joint_t)
      .def("get_target_translation", &Robot::get_target_translation)
      .def("get_target_rotation", &Robot::get_target_rotation)
      .def("get_current_pose", &Robot::get_current_pose)
      .def("get_current_joint_q", &Robot::get_current_joint_q)
      .def("get_current_joint_v", &Robot::get_current_joint_v)
      .def("get_current_joint_t", &Robot::get_current_joint_t)
      .def("get_current_translation", &Robot::get_current_translation)
      .def("get_current_rotation", &Robot::get_current_rotation)
      .def("get_current_end", &Robot::get_current_end)
      .def("get_current_joint_error_code", &Robot::get_current_joint_error_code)
      .def("get_current_joint_temperature", &Robot::get_current_joint_temperature)
      .def("get_motor_response_cnt", &Robot::get_motor_response_cnt)
      .def("get_sn", &Robot::get_sn)
      .def("set_target_pose",
           py::overload_cast<const std::vector<std::vector<double>> &, bool, double>(&Robot::set_target_pose),
           py::arg("target_pose"), py::arg("use_planning") = true, py::arg("time") = 1.)
      .def("set_target_pose",
           py::overload_cast<const std::vector<double> &, const std::vector<double> &, bool, double>(
               &Robot::set_target_pose),
           py::arg("target_translation"), py::arg("target_rotation"), py::arg("use_planning") = true,
           py::arg("time") = 1.)
      .def("set_target_translation", &Robot::set_target_translation, py::arg("target_translation"),
           py::arg("use_planning") = true, py::arg("time") = 1.)
      .def("add_target_translation", &Robot::add_target_translation, py::arg("target_d_translation"),
           py::arg("use_planning") = true, py::arg("time") = 1.)
      .def("add_target_relative_translation", &Robot::add_target_relative_translation, py::arg("target_d_translation"),
           py::arg("use_planning") = true, py::arg("time") = 1.)
      .def("set_target_rotation", &Robot::set_target_rotation, py::arg("target_rotation"),
           py::arg("use_planning") = true, py::arg("time") = 1.)
      .def("add_target_relative_rotation", &Robot::add_target_relative_rotation, py::arg("target_d_rotation"),
           py::arg("use_planning") = true, py::arg("time") = 1.)
      .def("set_target_vel", &Robot::set_target_vel, py::arg("target_vel"))
      .def("set_target_joint_q", &Robot::set_target_joint_q, py::arg("target_joint_q"), py::arg("use_planning") = true,
           py::arg("time") = 1.)
      .def("add_target_joint_q", &Robot::add_target_joint_q, py::arg("target_d_joint_q"),
           py::arg("use_planning") = true, py::arg("time") = 1.)
      .def("set_target_joint_v", &Robot::set_target_joint_v, py::arg("target_joint_v"))
      .def("add_target_joint_v", &Robot::add_target_joint_v, py::arg("target_d_joint_v"))
      .def("set_target_joint_t", &Robot::set_target_joint_t, py::arg("target_joint_t"))
      .def("set_target_end", &Robot::set_target_end, py::arg("target_end"))
      .def("record_start", &Robot::record_start)
      .def("record_stop", &Robot::record_stop)
      .def("record_replay", &Robot::record_replay)
      .def("record_save", &Robot::record_save, py::arg("filepath"))
      .def("record_load", &Robot::record_load, py::arg("filepath"))
      .def("gravity_compensation", &Robot::gravity_compensation)
      .def("stop_gravity_compensation", &Robot::stop_gravity_compensation)
      .def("set_max_current", &Robot::set_max_current, py::arg("max"), py::arg("second") = 1.);

  m.def("create_agent", &createAgent, py::arg("urdf_path") = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf",
        py::arg("direction") = "down", py::arg("can_interface") = "can0", py::arg("vel") = M_PI,
        py::arg("end_mode") = "newteacher", py::arg("constraint") = false);
};

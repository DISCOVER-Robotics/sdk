#include <airbot/modules/command/command_base.hpp>
#include <airbot/modules/controller/fk.hpp>
#include <airbot/modules/controller/fk_chain.hpp>
#include <airbot/modules/controller/ik.hpp>
#include <airbot/modules/controller/ik_chain.hpp>
#include <memory>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;
using namespace arm;

std::unique_ptr<Status> createStatus(ChainFKSolver* fksolver, ChainIKSolver* iksolver)
{
    return std::make_unique<Status>(std::unique_ptr<FKSolver>(fksolver), std::unique_ptr<IKSolver>(iksolver));
}

PYBIND11_MODULE(airbot, m)
{
    m.doc() = "airbot";
    m.def("createStatus", &createStatus, py::arg("fksolver"), py::arg("iksolver"));

    py::class_<ChainFKSolver>(m, "ChainFKSolver")
        .def(py::init<const std::string>());
    py::class_<ChainIKSolver>(m, "ChainIKSolver")
        .def(py::init<const std::string>());

    py::enum_<t_Mode>(m, "t_Mode")
        .value("POSITION", t_Mode::POSITION)
        .value("VELOCITY", t_Mode::VELOCITY)
        .value("TORQUE", t_Mode::TORQUE)
        .export_values();

    py::enum_<t_State>(m, "t_State")
        .value("END_POSE", t_State::END_POSE)
        .value("JOINT_POS", t_State::JOINT_POS)
        .value("JOINT_VEL", t_State::JOINT_VEL)
        .value("JOINT_TORQUE", t_State::JOINT_TORQUE)
        .export_values();

    py::class_<Status, std::unique_ptr<Status>>(m, "Status")
        .def("get_target_pose", &Status::get_target_pose)
        .def("get_target_joint_q", &Status::get_target_joint_q)
        .def("get_target_joint_v", &Status::get_target_joint_v)
        .def("get_target_joint_t", &Status::get_target_joint_t)
        .def("get_target_translation", &Status::get_target_translation)
        .def("get_target_rotation", &Status::get_target_rotation)
        .def("get_current_pose", &Status::get_current_pose)
        .def("get_current_joint_q", &Status::get_current_joint_q)
        .def("get_current_joint_v", &Status::get_current_joint_v)
        .def("get_current_joint_t", &Status::get_current_joint_t)
        .def("get_current_translation", &Status::get_current_translation)
        .def("get_current_rotation", &Status::get_current_rotation)
        .def("set_target_pose",
            py::overload_cast<const std::vector<std::vector<double>>&>(
                &Status::set_target_pose))
        .def("set_target_pose",
            py::overload_cast<const std::vector<double>&,
                const std::vector<double>&>(&Status::set_target_pose))
        .def("set_target_joint_q", &Status::set_target_joint_q, py::arg("target_joint_q"))
        .def("set_target_joint_v", &Status::set_target_joint_v, py::arg("target_joint_v"))
        .def("set_target_joint_t", &Status::set_target_joint_t, py::arg("target_joint_t"))
        .def("set_target_translation", &Status::set_target_translation,
            py::arg("target_translation"))
        .def("set_target_rotation", &Status::set_target_rotation,
            py::arg("target_rotation"))
        .def("add_target_joint_q", &Status::add_target_joint_q,
            py::arg("target_d_joint_q"))
        .def("add_target_joint_v", &Status::add_target_joint_v,
            py::arg("target_d_joint_v"))
        .def("add_target_translation", &Status::add_target_translation,
            py::arg("target_d_translation"));
};

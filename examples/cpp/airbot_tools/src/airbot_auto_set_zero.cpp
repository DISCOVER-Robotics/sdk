#include <ncurses.h>

#include <airbot/airbot.hpp>

#include "argparse/argparse.hpp"
#include "http.hpp"

namespace fs = std::filesystem;

const double ZERO_POINT[3] = {-3.15537, 0.182918, -0.107386};

int main(int argc, char **argv) {
  // argparse
  argparse::ArgumentParser program("airbot_auto_set_zero", AIRBOT_VERSION);
  program.add_description("A simple program to set zero point of AIRBOT Play.");
  program.add_argument("-m", "--master")
      .required()
      .default_value("can0")
      .help("Can device interface of the master arm.");
  program.add_argument("-u", "--urdf")
      .default_value(std::string())
      .help("Manually provided URDF path to override default paths.");
  program.add_argument("--forearm-type")
      .default_value("DM")
      .choices("DM", "OD")
      .help("The type of forearm. Available choices: \"DM\": Damiao motor, \"OD\": Self-developed motors");

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }
  std::string urdf_path = program.get<std::string>("--urdf");
  std::string master_can = program.get<std::string>("--master");
  std::string forearm_type = program.get<std::string>("--forearm-type");
  if (urdf_path == "") urdf_path = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf";

  // Synchronization
  std::unique_ptr<arm::Robot> robot;
  try {
    robot = std::make_unique<arm::Robot>(urdf_path, master_can, "down", M_PI / 2, "none", false, false, "DM", true);
  } catch (const std::runtime_error &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  double joint_pos[6];
  double read_past, read_now;
  robot->set_max_current({1, 2, 5, 10, 10, 10});
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  read_past = -20000;
  read_now = -2000;
  robot->set_target_joint_v({-0.5, 0, 0, 0, 0, 0});
  while (std::abs(read_now - read_past) > 0.001) {
    read_past = read_now;
    read_now = robot->get_current_joint_q()[0];
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  joint_pos[0] = read_now;

  read_past = -20000;
  read_now = -2000;
  robot->set_target_joint_v({0, 0.5, 0, 0, 0, 0});
  while (std::abs(read_now - read_past) > 0.001) {
    read_past = read_now;
    read_now = robot->get_current_joint_q()[1];
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  joint_pos[1] = read_now;

  read_past = -20000;
  read_now = -2000;
  robot->set_max_current({1, 1, 2, 10, 10, 10});
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  robot->set_target_joint_v({0, 0, -0.5, 0, 0, 0});
  while (std::abs(read_now - read_past) > 0.001) {
    read_past = read_now;
    read_now = robot->get_current_joint_q()[2];
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  joint_pos[2] = read_now;

  std::cerr << "Joint position: " << joint_pos[0] << ", " << joint_pos[1] << ", " << joint_pos[2] << std::endl;

  robot->set_max_current({10, 10, 10, 10, 10, 10});
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  robot->set_ignore_limit(true);
  robot->set_target_joint_q(
      {joint_pos[0] - ZERO_POINT[0], joint_pos[1] - ZERO_POINT[1], joint_pos[2] - ZERO_POINT[2], 0, 0, 0}, true);
  while (std::abs(robot->get_current_joint_q()[0] - (joint_pos[0] - ZERO_POINT[0])) > 0.001) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  robot->set_ignore_limit(false);
  robot->set_zero();

  return 0;
}

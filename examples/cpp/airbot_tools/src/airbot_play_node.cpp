#include <ncurses.h>

#include <airbot/airbot.hpp>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <shared_mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "argparse/argparse.hpp"
#include "http.hpp"

namespace fs = std::filesystem;

int main(int argc, char **argv)
{
  // argparse
  argparse::ArgumentParser program("airbot_play_node", AIRBOT_VERSION);
  program.add_description("A simple program to control AIRBOT Play via keyboard.");
  program.add_argument("-m", "--master")
      .required()
      .default_value("can0")
      .help("Can device interface of the master arm.");
  program.add_argument("-n", "--node")
      .default_value<std::vector<std::string>>({})
      .nargs(argparse::nargs_pattern::any)
      .help(
          "Can device interface of the following arm. Can use multiple times "
          "(multiple following arms). E.g., -n can1 can2");
  program.add_argument("-e", "--master-end-mode")
      .default_value("newteacher")
      .choices("teacher", "gripper", "yinshi", "newteacher", "none")
      .help(
          "The mode of the master arm end effector. Available choices: \n"
          "\"teacher\": The demonstrator equipped with Damiao motor \n"
          "\"gripper\": The gripper equipped with Damiao motor \n"
          "\"yinshi\": The Yinshi two-finger gripper \n"
          "\"newteacher\": The demonstrator equipped with self-developed motor \n"
          "\"none\": The arm is not equipped with end effector.");
  program.add_argument("-t", "--trajectory").default_value("").help("The trajectory file to replay");
  program.add_argument("-d", "--direction")
      .default_value("down")
      .choices("down", "left", "right")
      .help("The gravity direction. Useful for arms installed vertically");
  program.add_argument("-u", "--urdf")
      .default_value(std::string())
      .help("Manually provided URDF path to override default paths.");
  program.add_argument("--master-speed")
      .scan<'g', double>()
      .default_value(1.)
      .help("The joint speed of the master arm in ratio of PI.");
  program.add_argument("--constrained")
      .default_value(false)
      .implicit_value(true)
      .help(
          "Stop arm when going out of bounds in gravity compensation mode. "
          "False by default");
  program.add_argument("--real-time")
      .default_value(false)
      .implicit_value(true)
      .help(
          "Use maximum speed (master speed) for each motor when moving. False "
          "by default");

  try
  {
    program.parse_args(argc, argv);
  }
  catch (const std::exception &err)
  {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }
  std::string urdf_path = program.get<std::string>("--urdf");
  std::string master_can = program.get<std::string>("--master");
  std::vector<std::string> node_cans = program.get<std::vector<std::string>>("--node");
  std::string trajectory_path = program.get<std::string>("--trajectory");
  std::string master_end_mode = program.get<std::string>("--master-end-mode");
  std::string direction = program.get<std::string>("--direction");
  double master_speed = program.get<double>("--master-speed");
  bool constrained = program.get<bool>("--constrained");
  bool real_time = program.get<bool>("--real-time");
  if (urdf_path == "")
  {
    if (master_end_mode == "none")
      urdf_path = URDF_INSTALL_PATH + "airbot_play_v2_1.urdf";
    else
      urdf_path = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf";
  }

  // Synchronization
  std::unique_ptr<arm::Robot> robot;
  try
  {
    robot = std::make_unique<arm::Robot>(std::make_unique<arm::AnalyticFKSolver>(urdf_path),
                                    std::make_unique<arm::AnalyticIKSolver>(urdf_path),
                                    std::make_unique<arm::ChainIDSolver>(urdf_path, direction), master_can.c_str(), master_speed,
                                    master_end_mode, constrained, real_time);
  }
  catch (const std::runtime_error &e)
  {
    std::cerr << e.what() << '\n';
    endwin();
    return 1;
  }

  robot->record_load(trajectory_path);

  // init terminal
  filter();
  initscr();            // Initialize ncurses
  raw();                // Disable line buffering
  keypad(stdscr, TRUE); // Enable special keys
  noecho();             // Disable echoing of characters
  timeout(2);

  // Manipulation of Master arm
  auto from_base = true;
  auto step = 0.01;
  auto angle_step = 0.1;
  auto gripper_state = false;
  double x, y, z, w;
  while (1)
  {
    int ch = getch();
    uint32_t end_snap_signal = robot->get_end_snap_signal();
    uint32_t base_snap_signal = robot->get_base_snap_signal();
    if (end_snap_signal || base_snap_signal)
    {
      robot->end_snap_signal_handle(end_snap_signal);
      robot->base_snap_signal_handle(base_snap_signal);
    }
    else
    {
      switch (ch)
      {
      case '1':
        robot->add_target_joint_q({angle_step, 0, 0, 0, 0, 0});
        break;
      case '2':
        robot->add_target_joint_q({-angle_step, 0, 0, 0, 0, 0});
        break;
      case '3':
        robot->add_target_joint_q({0, angle_step, 0, 0, 0, 0});
        break;
      case '4':
        robot->add_target_joint_q({0, -angle_step, 0, 0, 0, 0});
        break;
      case '5':
        robot->add_target_joint_q({0, 0, angle_step, 0, 0, 0});
        break;
      case '6':
        robot->add_target_joint_q({0, 0, -angle_step, 0, 0, 0});
        break;
      case '7':
        robot->add_target_joint_q({0, 0, 0, angle_step, 0, 0});
        break;
      case '8':
        robot->add_target_joint_q({0, 0, 0, -angle_step, 0, 0});
        break;
      case '9':
        robot->add_target_joint_q({0, 0, 0, 0, angle_step, 0});
        break;
      case '0':
        robot->add_target_joint_q({0, 0, 0, 0, -angle_step, 0});
        break;
      case '-':
        robot->add_target_joint_q({0, 0, 0, 0, 0, angle_step});
        break;
      case '=':
        robot->add_target_joint_q({0, 0, 0, 0, 0, -angle_step});
        break;
      case 'w':
        if (from_base)
        {
          robot->add_target_translation({step, 0, 0});
        }
        else
        {
          robot->add_target_relative_translation({step, 0, 0});
        }
        break;
      case 's':
        if (from_base)
        {
          robot->add_target_translation({-step, 0, 0});
        }
        else
        {
          robot->add_target_relative_translation({-step, 0, 0});
        }
        break;
      case 'a':
        if (from_base)
        {
          robot->add_target_translation({0, step, 0});
        }
        else
        {
          robot->add_target_relative_translation({0, step, 0});
        }
        break;
      case 'd':
        if (from_base)
        {
          robot->add_target_translation({0, -step, 0});
        }
        else
        {
          robot->add_target_relative_translation({0, -step, 0});
        }
        break;
      case 'q':
        if (from_base)
        {
          robot->add_target_translation({0, 0, step});
        }
        else
        {
          robot->add_target_relative_translation({0, 0, step});
        }
        break;
      case 'e':
        if (from_base)
        {
          robot->add_target_translation({0, 0, -step});
        }
        else
        {
          robot->add_target_relative_translation({0, 0, -step});
        }
        break;
      case 'r':
        from_base = !from_base;
        break;
      case 'j':
        KDL::Rotation::RPY(0, 0, angle_step).GetQuaternion(x, y, z, w);
        robot->add_target_relative_rotation({x, y, z, w});
        break;
      case 'l':
        KDL::Rotation::RPY(0, 0, -angle_step).GetQuaternion(x, y, z, w);
        robot->add_target_relative_rotation({x, y, z, w});
        break;
      case 'i':
        KDL::Rotation::RPY(0, angle_step, 0).GetQuaternion(x, y, z, w);
        robot->add_target_relative_rotation({x, y, z, w});
        break;
      case 'k':
        KDL::Rotation::RPY(0, -angle_step, 0).GetQuaternion(x, y, z, w);
        robot->add_target_relative_rotation({x, y, z, w});
        break;
      case 'u':
        KDL::Rotation::RPY(angle_step, 0, 0).GetQuaternion(x, y, z, w);
        robot->add_target_relative_rotation({x, y, z, w});
        break;
      case 'o':
        KDL::Rotation::RPY(-angle_step, 0, 0).GetQuaternion(x, y, z, w);
        robot->add_target_relative_rotation({x, y, z, w});
        break;
      case '`':
        robot->set_target_joint_q({0., 0., 0., 0., 0., 0.});
        break;

      case 'm':
        robot->record_save("records/" + std::to_string(arm::get_timestamp()) + ".json");
        break;
      case '[':
        robot->set_target_end(0);
        break;
      case ']':
        robot->set_target_end(1);
        break;
      case '\\':
        robot->alter_logging();
        break;
      default:
        break;
      }
      if (ch == 'z' || ch == 3)
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  endwin();
  return 0;
}

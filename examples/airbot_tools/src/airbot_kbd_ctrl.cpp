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
constexpr const double MOVING_VEL = 0.2;
int main(int argc, char **argv) {
  // argparse
  argparse::ArgumentParser program("airbot_kbd_ctrl", AIRBOT_VERSION);
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
      .choices("teacher", "gripper", "yinshi", "newteacher", "none", "teacherv2", "encoder")
      .help(
          "The mode of the master arm end effector. Available choices: \n"
          "\"teacher\": The demonstrator equipped with Damiao motor \n"
          "\"gripper\": The gripper equipped with Damiao motor \n"
          "\"yinshi\": The Yinshi two-finger gripper \n"
          "\"newteacher\": The demonstrator equipped with self-developed "
          "motor \n"
          "\"teacherv2\": The V2 version of demonstrator equipped with self-developed motor \n"
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
      .help("The maximum joint speed of the master arm in ratio of PI.");
  program.add_argument("--constrained")
      .default_value(false)
      .implicit_value(true)
      .help("Stop arm when going out of bounds in gravity compensation mode. False by default");
  program.add_argument("--bigarm-type")
      .default_value("OD")
      .choices("DM", "OD", "encoder")
      .help(
          "The type of big arm. Available choices: \"DM\": Damiao motor, \"OD\": Self-developed motors, \"encoder\": "
          "Self-developed encoders");
  program.add_argument("--forearm-type")
      .default_value("DM")
      .choices("DM", "OD", "encoder")
      .help(
          "The type of forearm. Available choices: \"DM\": Damiao motor, \"OD\": Self-developed motors, \"encoder\": "
          "Self-developed encoders");

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception &err) {
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
  double master_speed = program.get<double>("--master-speed") * M_PI;
  bool constrained = program.get<bool>("--constrained");
  std::string bigarm_type = program.get<std::string>("--bigarm-type");
  std::string forearm_type = program.get<std::string>("--forearm-type");
  if (urdf_path == "") {
    if (master_end_mode == "none")
      urdf_path = URDF_INSTALL_PATH + "airbot_play.urdf";
    else
      urdf_path = URDF_INSTALL_PATH + "airbot_play_with_gripper.urdf";
  }

  // Synchronization
  std::shared_ptr<arm::Robot<6>> robot;
  try {
    robot = std::make_shared<arm::Robot<6>>(urdf_path, master_can, direction, master_speed, master_end_mode,
                                            bigarm_type, forearm_type);
  } catch (const std::runtime_error &e) {
    std::cerr << e.what() << '\n';
    endwin();
    return 1;
  }

  robot->record_load(trajectory_path);

  // init terminal
  initscr();             // Initialize ncurses
  raw();                 // Disable line buffering
  keypad(stdscr, TRUE);  // Enable special keys
  noecho();              // Disable echoing of characters
  timeout(2);

  // Manipulation of Master arm
  auto from_base = true;
  auto step = 0.01;
  auto angle_step = M_PI / 10;
  auto use_planning = true;
  auto gripper_state = false;
  bool recording = false;
  auto logging = false;
  double x, y, z, w;
  move(0, 0);
  printw("AIRBOT Kerboard Control Ver. %s Keyboard Reference:", AIRBOT_VERSION);
  refresh();
  move(1, 0);
  printw("1 / 2: Motor 1 left / right    3 / 4: Motor 2 backward / forward    5 / 6: Motor 3 upward / downward");
  refresh();
  move(2, 0);
  printw("7 / 8: Motor 4 right / left    9 / 0: Motor 5 left / right          - / =: Motor 6 right / left");
  refresh();
  move(3, 0);
  printw("[ / ]: (If gripper connected) gripper close / open                  `:     Return to zero point");
  refresh();
  move(4, 0);
  printw("w / s / a / d / q / e: move the end effector forward / backward / left / right / upward / downward in ");
  refresh();
  move(5, 0);
  printw("                       base frame or end frame (base frame by default)");
  refresh();
  move(6, 0);
  printw("r: alter the frame used by w/a/s/d/q/e, base or end frame    t: change the moving step of commands");
  refresh();
  move(7, 0);
  printw("j / l / i / k / u / o: rotate the end effector in roll / pitch / yaw axis");
  refresh();
  move(8, 0);
  printw(
      "/: Reset error state    x: Enter manual mode (gravity compensation)   c: Enter online mode (command control)");
  refresh();
  move(9, 0);
  printw(
      "v: Enter offline mode (replay)      b: Start / stop recording (in manual mode)       n: Start replaying (in "
      "offline mode)");
  refresh();
  move(10, 0);
  printw("Ctrl + C / z: exit");
  refresh();
  while (1) {
    int ch = getch();
    switch (ch) {
      case '1':
        robot->add_target_joint_q({angle_step, 0, 0, 0, 0, 0}, use_planning, MOVING_VEL);
        break;
      case '2':
        robot->add_target_joint_q({-angle_step, 0, 0, 0, 0, 0}, use_planning, MOVING_VEL);
        break;
      case '3':
        robot->add_target_joint_q({0, angle_step, 0, 0, 0, 0}, use_planning, MOVING_VEL);
        break;
      case '4':
        robot->add_target_joint_q({0, -angle_step, 0, 0, 0, 0}, use_planning, MOVING_VEL);
        break;
      case '5':
        robot->add_target_joint_q({0, 0, angle_step, 0, 0, 0}, use_planning, MOVING_VEL);
        break;
      case '6':
        robot->add_target_joint_q({0, 0, -angle_step, 0, 0, 0}, use_planning, MOVING_VEL);
        break;
      case '7':
        robot->add_target_joint_q({0, 0, 0, angle_step, 0, 0}, use_planning, MOVING_VEL);
        break;
      case '8':
        robot->add_target_joint_q({0, 0, 0, -angle_step, 0, 0}, use_planning, MOVING_VEL);
        break;
      case '9':
        robot->add_target_joint_q({0, 0, 0, 0, angle_step, 0}, use_planning, MOVING_VEL);
        break;
      case '0':
        robot->add_target_joint_q({0, 0, 0, 0, -angle_step, 0}, use_planning, MOVING_VEL);
        break;
      case '-':
        robot->add_target_joint_q({0, 0, 0, 0, 0, angle_step}, use_planning, MOVING_VEL);
        break;
      case '=':
        robot->add_target_joint_q({0, 0, 0, 0, 0, -angle_step}, use_planning, MOVING_VEL);
        break;
      case 'w':
        if (from_base) {
          robot->add_target_translation({step, 0, 0}, use_planning, MOVING_VEL);
        } else {
          robot->add_target_relative_translation({0, 0, step}, use_planning, MOVING_VEL);
        }
        break;
      case 's':
        if (from_base) {
          robot->add_target_translation({-step, 0, 0}, use_planning, MOVING_VEL);
        } else {
          robot->add_target_relative_translation({0, 0, -step}, use_planning, MOVING_VEL);
        }
        break;
      case 'a':
        if (from_base) {
          robot->add_target_translation({0, step, 0}, use_planning, MOVING_VEL);
        } else {
          robot->add_target_relative_translation({0, step, 0}, use_planning, MOVING_VEL);
        }
        break;
      case 'd':
        if (from_base) {
          robot->add_target_translation({0, -step, 0}, use_planning, MOVING_VEL);
        } else {
          robot->add_target_relative_translation({0, -step, 0}, use_planning, MOVING_VEL);
        }
        break;
      case 'q':
        if (from_base) {
          robot->add_target_translation({0, 0, step}, use_planning, MOVING_VEL);
        } else {
          robot->add_target_relative_translation({-step, 0, 0}, use_planning, MOVING_VEL);
        }
        break;
      case 'e':
        if (from_base) {
          robot->add_target_translation({0, 0, -step}, use_planning, MOVING_VEL);
        } else {
          robot->add_target_relative_translation({step, 0, 0}, use_planning, MOVING_VEL);
        }
        break;
      case 'r':
        from_base = !from_base;
        break;
      case 't':
        if (step == 0.01) {
          step = 0.1;
        } else {
          step = 0.01;
        }
        if (angle_step == M_PI / 10) {
          angle_step = M_PI / 100;
        } else {
          angle_step = M_PI / 10;
        }
        break;
      case 'j':
        KDL::Rotation::RPY(0, 0, angle_step).GetQuaternion(x, y, z, w);
        robot->add_target_relative_rotation({x, y, z, w}, use_planning, MOVING_VEL);
        break;
      case 'l':
        KDL::Rotation::RPY(0, 0, -angle_step).GetQuaternion(x, y, z, w);
        robot->add_target_relative_rotation({x, y, z, w}, use_planning, MOVING_VEL);
        break;
      case 'i':
        KDL::Rotation::RPY(0, angle_step, 0).GetQuaternion(x, y, z, w);
        robot->add_target_relative_rotation({x, y, z, w}, use_planning, MOVING_VEL);
        break;
      case 'k':
        KDL::Rotation::RPY(0, -angle_step, 0).GetQuaternion(x, y, z, w);
        robot->add_target_relative_rotation({x, y, z, w}, use_planning, MOVING_VEL);
        break;
      case 'u':
        KDL::Rotation::RPY(angle_step, 0, 0).GetQuaternion(x, y, z, w);
        robot->add_target_relative_rotation({x, y, z, w}, use_planning, MOVING_VEL);
        break;
      case 'o':
        KDL::Rotation::RPY(-angle_step, 0, 0).GetQuaternion(x, y, z, w);
        robot->add_target_relative_rotation({x, y, z, w}, use_planning, MOVING_VEL);
        break;
      case '`':
        robot->set_target_joint_q({0., 0., 0., 0., 0., 0.}, use_planning, MOVING_VEL);
        break;
      case 'm':
        robot->record_save("records/" + get_timestring() + ".json");
        break;
      case '[':
        robot->set_target_end(0);
        break;
      case ']':
        robot->set_target_end(1);
        break;
      case '/':
        robot->reset_error();
        break;
      case 'x':
        robot->manual_mode();
        break;
      case 'c':
        robot->online_mode();
        break;
      case 'v':
        robot->offline_mode();
        break;
      case 'b':
        if (!recording) {
          robot->record_start();
          recording = true;
        } else {
          robot->record_stop();
          recording = false;
        }
        break;
      case 'n':
        robot->replay_start();
        break;
      case ' ':
        if (!logging) {
          logging = true;
          robot->logging(10);
        } else {
          logging = false;
          robot->logging(0);
        }
        break;
      default:
        break;
    }
    if (ch == 'z' || ch == 3) break;

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  endwin();
  return 0;
}

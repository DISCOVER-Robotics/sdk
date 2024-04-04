#include <ncurses.h>
#undef OK
#include <airbot/airbot.hpp>

#include "argparse/argparse.hpp"

int main(int argc, char **argv) {
  // argparse
  argparse::ArgumentParser program("airbot_sync", AIRBOT_VERSION);
  program.add_argument("-u", "--urdf")
      .default_value(std::string())
      .help("Manually provided URDF path to override default paths.");
  program.add_argument("-m", "--master")
      .required()
      .default_value("can0")
      .help("Can device interface of the master arm.");
  program.add_argument("-n", "--node")
      .required()
      .default_value("can1")
      .help("Can device interface of the following arm.");
  program.add_argument("-d", "--direction")
      .default_value("down")
      .choices("down", "left", "right")
      .help("The gravity direction. Useful for arms installed vertically");
  program.add_argument("--master-end-mode")
      .default_value("newteacher")
      .choices("newteacher", "teacher", "gripper", "yinshi")
      .help(
          "The mode of the master arm end effector. Available choices: \"teacher\", \"gripper\", \"yinshi\", "
          "\"newteacher\"");
  program.add_argument("--follower-end-mode")
      .default_value("gripper")
      .choices("newteacher", "teacher", "gripper", "yinshi")
      .help(
          "The mode of the follower arm end effector. Available choices: \"teacher\", \"gripper\", \"yinshi\", "
          "\"newteacher\"");
  program.add_argument("--master-speed")
      .scan<'g', double>()
      .default_value(1.)
      .help("The joint speed of the master arm in percentage of PI.");
  program.add_argument("--follower-speed")
      .scan<'g', double>()
      .default_value(3.)
      .help("The joint speed of the follower arm in percentage of PI.");

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  std::string urdf_path = program.get<std::string>("--urdf");
  std::string master_can = program.get<std::string>("--master");
  std::string node_can = program.get<std::string>("--node");
  std::string master_end_mode = program.get<std::string>("--master-end-mode");
  std::string follower_end_mode = program.get<std::string>("--follower-end-mode");
  std::string direction = program.get<std::string>("--direction");
  double master_speed = program.get<double>("--master-speed");
  double follower_speed = program.get<double>("--follower-speed");
  if (urdf_path == "") {
    if (master_end_mode == "none")
      urdf_path = URDF_INSTALL_PATH + "airbot_play_v2_1.urdf";
    else
      urdf_path = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf";
  }

  bool running = true;
  std::shared_mutex mutex_;
  std::vector<std::thread> threads;

  auto leader = std::make_unique<arm::Robot>(urdf_path, master_can, direction, master_speed, master_end_mode);
  auto follower = std::make_unique<arm::Robot>(urdf_path, node_can, direction, follower_speed, follower_end_mode);

  threads.emplace_back(std::thread([&]() {
    while (true) {
      std::shared_lock<std::shared_mutex> lock(mutex_);
      if (!running) break;
      lock.unlock();
      follower->set_target_joint_q(leader->get_current_joint_q(), false);
      follower->set_target_end(leader->get_current_end());
    }
  }));

  // main: keyborad control
  // init terminal
  filter();
  initscr();             // Initialize ncurses
  raw();                 // Disable line buffering
  keypad(stdscr, TRUE);  // Enable special keys
  noecho();              // Disable echoing of characters
  timeout(2);
  bool gravity_compensation_flag = false;
  while (1) {
    int ch = getch();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    switch (ch) {
      case '[':
        leader->set_target_end(0);
        break;
      case ']':
        leader->set_target_end(1);
        break;
      default:
        break;
    }
    if (ch == 'z' || ch == 3) {
      break;
    }
  }
  endwin();

  std::unique_lock<std::shared_mutex> lock(mutex_);
  running = false;
  lock.unlock();

  for (auto &&i : threads) i.join();
  auto reset_leader = std::thread([&]() { leader.reset(nullptr); });
  auto reset_follower = std::thread([&]() { follower.reset(nullptr); });
  reset_leader.join();
  reset_follower.join();
  return 0;
}

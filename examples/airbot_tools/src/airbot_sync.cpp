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
  program.add_argument("-m", "--leader")
      .required()
      .default_value("can0")
      .help("Can device interface of the master arm.");
  program.add_argument("-n", "--follower")
      .required()
      .default_value("can1")
      .help("Can device interface of the following arm.");
  program.add_argument("-d", "--direction")
      .default_value("down")
      .choices("down", "left", "right")
      .help("The gravity direction. Useful for arms installed vertically");
  program.add_argument("--leader-end-mode")
      .default_value("newteacher")
      .choices("newteacher", "teacher", "gripper", "yinshi", "none", "teacherv2")
      .help(
          "The mode of the master arm end effector. Available choices: \n"
          "\"teacher\": The demonstrator equipped with Damiao motor \n"
          "\"gripper\": The gripper equipped with Damiao motor \n"
          "\"yinshi\": The Yinshi two-finger gripper \n"
          "\"newteacher\": The demonstrator equipped with self-developed "
          "motor \n"
          "\"teacherv2\": The V2 version of demonstrator equipped with self-developed motor \n"
          "\"none\": The arm is not equipped with end effector.");
  program.add_argument("--follower-end-mode")
      .default_value("gripper")
      .choices("newteacher", "teacher", "gripper", "yinshi", "none", "teacherv2")
      .help(
          "The mode of the master arm end effector. Available choices: \n"
          "\"teacher\": The demonstrator equipped with Damiao motor \n"
          "\"gripper\": The gripper equipped with Damiao motor \n"
          "\"yinshi\": The Yinshi two-finger gripper \n"
          "\"newteacher\": The demonstrator equipped with self-developed "
          "motor \n"
          "\"teacherv2\": The V2 version of demonstrator equipped with self-developed motor \n"
          "\"none\": The arm is not equipped with end effector.");
  program.add_argument("--leader-speed")
      .scan<'g', double>()
      .default_value(1 * M_PI)
      .help("The joint speed of the master arm in percentage of PI.");
  program.add_argument("--follower-speed")
      .scan<'g', double>()
      .default_value(3 * M_PI)
      .help("The joint speed of the follower arm in percentage of PI.");
  program.add_argument("--leader-armtype")
      .default_value("DM")
      .choices("DM", "OD")
      .help("The type of forearm. Available choices: \"DM\": Damiao motor, \"OD\": Self-developed motors");
  program.add_argument("--follower-armtype")
      .default_value("DM")
      .choices("DM", "OD")
      .help("The type of forearm. Available choices: \"DM\": Damiao motor, \"OD\": Self-developed motors");
  program.add_argument("--mit").default_value(false).implicit_value(true).help("Enable force feedback control.");
  try {
    program.parse_args(argc, argv);
  } catch (const std::exception &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  std::string urdf_path = program.get<std::string>("--urdf");
  std::string master_can = program.get<std::string>("--leader");
  std::string node_can = program.get<std::string>("--follower");
  std::string master_end_mode = program.get<std::string>("--leader-end-mode");
  std::string follower_end_mode = program.get<std::string>("--follower-end-mode");
  std::string direction = program.get<std::string>("--direction");
  std::string leader_armtype = program.get<std::string>("--leader-armtype");
  std::string follower_armtype = program.get<std::string>("--follower-armtype");
  double master_speed = program.get<double>("--leader-speed");
  double follower_speed = program.get<double>("--follower-speed");
  bool force_feedback = program.get<bool>("--mit");
  if (urdf_path == "") {
    if (master_end_mode == "none")
      urdf_path = URDF_INSTALL_PATH + "airbot_play_v2_1.urdf";
    else
      urdf_path = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf";
  }

  bool running = true;
  std::shared_mutex mutex_;
  std::vector<std::thread> threads;

  auto leader =
      std::make_unique<arm::Robot<6>>(urdf_path, master_can, direction, master_speed, master_end_mode, leader_armtype);
  auto follower = std::make_unique<arm::Robot<6>>(urdf_path, node_can, direction, follower_speed, follower_end_mode,
                                                  follower_armtype);

  Joints<6> follower_joint_kps = {100.0f, 100.0f, 100.0f, 10.0f, 10.0f, 10.0f};
  Joints<6> follower_joint_kds = {1.0f, 1.0f, 1.0f, 0.02f, 0.02f, 0.02f};
  Joints<6> follower_joint_p_gains = {7.f, 7.f, 7.f, .5f, 1.f, 1.f};
  double follower_gain = 2.5;
  if (master_end_mode != "none") {
    follower_joint_kps = {10.0f, 35.0f, 55.0f, 7.5f, 7.5f, 7.5f};
    follower_joint_kds = {1.5f, 1.75f, 1.5f, 0.25f, 0.25f, 0.25f};
    follower_joint_p_gains = {2.5f, 2.5f, 2.5f, 0.5f, 0.5f, 0.5f};
  }

  float MAX_EFFORT = 2.f;
  double clip_joint_range[6];
  for (size_t i = 0; i < 6; i++) {
    clip_joint_range[i] = 4. * MAX_EFFORT / (follower_joint_kps[i] + 1e-6);
  }
  threads.emplace_back(std::thread([&]() {
    while (true) {
      std::shared_lock<std::shared_mutex> lock(mutex_);
      if (!running) break;
      lock.unlock();
      if (force_feedback) {
        auto q_teacher = leader->get_current_joint_q();
        auto q_follower = follower->get_current_joint_q();

        float clip_joint_diff[6];
        double joint_diff[6];
        Joints<6> set_q_follower;
        for (size_t i_m = 0; i_m < 6; i_m++) {
          joint_diff[i_m] = q_teacher[i_m] - q_follower[i_m];
          clip_joint_diff[i_m] =
              static_cast<float>(std::clamp(joint_diff[i_m], -clip_joint_range[i_m], clip_joint_range[i_m]));
          set_q_follower[i_m] = q_follower[i_m] + clip_joint_diff[i_m];
        }
        follower->set_target_joint_mit(set_q_follower, {0, 0, 0, 0, 0, 0}, follower_joint_kps, follower_joint_kds);
        follower->set_target_end(leader->get_current_end());
      } else {
        follower->set_target_joint_q(leader->get_current_joint_q(), false, follower_speed);
        follower->set_target_end(leader->get_current_end());
      }
    }
  }));

  // main: keyborad control
  // init terminal
  initscr();             // Initialize ncurses
  raw();                 // Disable line buffering
  keypad(stdscr, TRUE);  // Enable special keys
  noecho();              // Disable echoing of characters
  timeout(2);
  bool recording = false;
  move(0, 0);
  printw("AIRBOT Sync Control Ver. %s Keyboard Reference:", AIRBOT_VERSION);
  refresh();
  move(1, 0);
  printw("[ / ]: (If gripper connected) gripper close / open                  `:     Return to zero point");
  refresh();
  move(2, 0);
  printw(
      "/: Reset error state    x: Enter manual mode (gravity compensation)   c: Enter online mode (command control)");
  refresh();
  move(3, 0);
  printw(
      "v: Enter offline mode (replay)      b: Start / stop recording (in manual mode)       n: Start replaying (in "
      "offline mode)");
  refresh();
  move(4, 0);
  printw("Ctrl + C / z: exit");

  while (1) {
    int ch = getch();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    switch (ch) {
      case '`':
        leader->set_target_joint_q({0., 0., 0., 0., 0., 0.}, true);
        break;
      case '[':
        leader->set_target_end(0);
        break;
      case ']':
        leader->set_target_end(1);
        break;
      case '/':
        leader->reset_error();
        follower->reset_error();
        break;
      case 'x':
        leader->manual_mode();
        break;
      case 'c':
        leader->online_mode();
        break;
      case 'v':
        leader->offline_mode();
        break;
      case 'b':
        if (!recording) {
          leader->record_start();
          recording = true;
        } else {
          leader->record_stop();
          recording = false;
        }
        break;
      case 'n':
        for (int i = 0; i < 100; i++) {
          leader->replay_start();
          std::this_thread::sleep_for(std::chrono::seconds(5));
        }
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

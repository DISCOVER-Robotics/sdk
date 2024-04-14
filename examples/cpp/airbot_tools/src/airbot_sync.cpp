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
      .choices("newteacher", "teacher", "gripper", "yinshi", "none")
      .help(
          "The mode of the master arm end effector. Available choices: \"teacher\", \"gripper\", \"yinshi\", "
          "\"newteacher\"");
  program.add_argument("--follower-end-mode")
      .default_value("gripper")
      .choices("newteacher", "teacher", "gripper", "yinshi", "none")
      .help(
          "The mode of the follower arm end effector. Available choices: \"teacher\", \"gripper\", \"yinshi\", "
          "\"newteacher\"");
  program.add_argument("--master-speed")
      .scan<'g', double>()
      .default_value(1 * M_PI)
      .help("The joint speed of the master arm in percentage of PI.");
  program.add_argument("--follower-speed")
      .scan<'g', double>()
      .default_value(3 * M_PI)
      .help("The joint speed of the follower arm in percentage of PI.");
  program.add_argument("--force-feedback")
      .default_value(false)
      .implicit_value(true)
      .help("Enable force feedback control.");
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
  bool force_feedback = program.get<bool>("--force-feedback");
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

  vector<double> follower_joint_kps = vector<double>{75.0f, 75.0f, 75.0f, 3.0f, 3.0f, 3.0f};
  vector<double> follower_joint_kds = vector<double>{1.0f, 1.0f, 1.0f, 0.02f, 0.02f, 0.02f};
  vector<double> follower_joint_p_gains = vector<double>{5.f, 5.f, 5.f, 0.2f, 0.2f, 0.2f};

  float MAX_EFFORT = 2.f;
  double clip_joint_range[6];
  for (size_t i = 0; i < 6; i++) {
    clip_joint_range[i] = 2. * MAX_EFFORT / (follower_joint_kps[i] + 1e-6);
  }
  if (force_feedback) {
    follower->set_mit_params(follower_joint_kps, follower_joint_kds);
  }

  threads.emplace_back(std::thread([&]() {
    while (true) {
      std::shared_lock<std::shared_mutex> lock(mutex_);
      if (!running) break;
      lock.unlock();
      if (force_feedback) {
        auto t_tau = leader->get_target_joint_t();
        auto t_tau_follower = follower->get_target_joint_t();
        auto q_teacher = leader->get_current_joint_q();
        auto q_follower = follower->get_current_joint_q();

        float clip_joint_diff[6];
        double joint_diff[6];
        vector<double> set_q_follower(6);
        for (size_t i_m = 0; i_m < 6; i_m++) {
          joint_diff[i_m] = q_teacher[i_m] - q_follower[i_m];
          clip_joint_diff[i_m] =
              static_cast<float>(std::clamp(joint_diff[i_m], -clip_joint_range[i_m], clip_joint_range[i_m]));
          set_q_follower[i_m] = q_follower[i_m] + clip_joint_diff[i_m];
        }
        // 只有1控1的时候 力反馈
        std::vector<double> fb_force;
        for (size_t i_m = 0; i_m < 6; i_m++) {
          fb_force.push_back(1.5 * follower_joint_p_gains[i_m] * joint_diff[i_m]);
        }
        leader->set_mit_feedback(fb_force);
        follower->set_target_joint_qt(set_q_follower, t_tau_follower);
        vector<double> follower_mit_feedback;
        for (size_t i_m = 0; i_m < 6; i_m++) {
          follower_mit_feedback.push_back(-follower_joint_p_gains[i_m] * clip_joint_diff[i_m]);
        }
        follower->set_mit_feedback(follower_mit_feedback);
      } else {
        follower->set_target_joint_q(leader->get_current_joint_q(), false);
        follower->set_target_end(leader->get_current_end());
      }
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

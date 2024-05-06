#include <ncurses.h>
#undef OK

#include <airbot/airbot.hpp>
#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <queue>
#include <shared_mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "argparse/argparse.hpp"
#include "camera/webcam.hpp"

namespace fs = std::filesystem;

void print(std::string s, int w = 0, int h = 0) {
  move(w, h);
  refresh();
  clrtoeol();
  printw(s.c_str());
}

cv::Mat mergeRGBHorizontally(const std::vector<cv::Mat>& mats) {
  // Ensure that input images are all not empty
  std::vector<cv::Mat> mats_;
  for (auto image : mats) {
    if (image.empty())
      return cv::Mat();
    else
      mats_.emplace_back(image);
  }

  // Get the number of channels
  int numChannels = mats_[0].channels();
  if (numChannels != 3) {
    std::cerr << "Input Mats must be RGB Mats!" << std::endl;
    return cv::Mat();
  }

  // Downsample each Mat by a factor of 2
  std::vector<cv::Mat> downsampledMats;
  for (const auto& mat : mats_) {
    cv::Mat downsampledMat;
    cv::resize(mat, downsampledMat, cv::Size(), 0.5, 0.5);  // Downsample by a factor of 2
    downsampledMats.push_back(downsampledMat);
  }

  // Split channels
  std::vector<cv::Mat> channels(downsampledMats.size() * numChannels);
  for (size_t i = 0; i < downsampledMats.size(); ++i) {
    std::vector<cv::Mat> tempChannels;
    cv::split(downsampledMats[i], tempChannels);
    for (int j = 0; j < numChannels; ++j) {
      channels[i * numChannels + j] = tempChannels[j];
    }
  }

  // Merge channels horizontally
  std::vector<cv::Mat> mergedChannels(numChannels);
  for (int i = 0; i < numChannels; ++i) {
    std::vector<cv::Mat> tempChannels;
    for (size_t j = 0; j < downsampledMats.size(); ++j) {
      tempChannels.push_back(channels[j * numChannels + i]);
    }
    cv::hconcat(tempChannels, mergedChannels[i]);
  }

  // Merge RGB Mats
  cv::Mat mergedRGB;
  cv::merge(mergedChannels, mergedRGB);

  return mergedRGB;
}

void createDirIfNotExists(const fs::path& path) {
  if (!fs::exists(path)) {
    try {
      if (fs::create_directories(path)) print("Directory created: " + std::string(path), 2, 0);
    } catch (const fs::filesystem_error& e) {
      std::cerr << e.what() << std::endl;
    };
  } else {
    print("Directory already exists: " + std::string(path), 2, 0);
  }
}

int get_key() {
  int ch = getch();
  return ch;
}

inline time_t get_timestamp() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
      .count();
}

int main(int argc, char** argv) {
  // argparse
  argparse::ArgumentParser program("airbot_demonstrate_dual", AIRBOT_VERSION);
  program.add_argument("-u", "--urdf")
      .default_value(
          std::getenv("URDF_PATH")
              ? std::string(std::getenv("URDF_PATH"))
              : std::string("/usr/local/share/airbot_play/airbot_play_v2_1/urdf/airbot_play_v2_1_with_gripper.urdf"))
      .help(
          "URDF file for describing the arm. If not provided, check and read from the "
          "environment variable URDF_PATH or load from deb install path.");
  program.add_argument("-ml", "--master_left")
      .required()
      .default_value("can0")
      .help("Can device interface of the master arm.");
  program.add_argument("-nl", "--node_left")
      .required()
      .default_value("can1")
      .help("Can device interface of the following arm.");
  program.add_argument("-mr", "--master_right")
      .required()
      .default_value("can2")
      .help("Can device interface of the master arm.");
  program.add_argument("-nr", "--node_right")
      .required()
      .default_value("can3")
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
  program.add_argument("-f", "--frequency")
      .scan<'i', int>()
      .default_value(15)
      .help("The frequency of the recording action");
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
  program.add_argument("-c", "--camera")
      .default_value<std::vector<std::string>>({})
      .append()
      .help("Camera devices indices. Can use multiple times (multiple cameras). E.g., -c 0 -c 1");
  program.add_argument("-se", "--start-episode")
      .default_value(0)
      .scan<'d', int>()
      .help("The start episode number for saving data. Default is 0.");
  program.add_argument("-tn", "--task-name")
      .default_value("test_task")
      .help("The name of the task which will be used as the folder name.");
  program.add_argument("-mts", "--max-time-steps")
      .default_value(1000)
      .scan<'d', int>()
      .help("The max time steps to collect data.");
  program.add_argument("-sjpl", "--start-joint-pos-left")
      .default_value<std::vector<double>>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
      .scan<'g', double>()
      .nargs(7)
      .help("Start joint positions for data collection");
  program.add_argument("-sjpr", "--start-joint-pos-right")
      .default_value<std::vector<double>>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
      .scan<'g', double>()
      .nargs(7)
      .help("Start joint positions for data collection");

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  std::string urdf_path = program.get<std::string>("--urdf");
  std::string master_can_left = program.get<std::string>("--master_left");
  std::string master_can_right = program.get<std::string>("--master_right");
  std::string node_can_left = program.get<std::string>("--node_left");
  std::string node_can_right = program.get<std::string>("--node_right");
  std::string master_end_mode = program.get<std::string>("--master-end-mode");
  std::string follower_end_mode = program.get<std::string>("--follower-end-mode");
  std::string direction = program.get<std::string>("--direction");
  double master_speed = program.get<double>("--master-speed");
  double follower_speed = program.get<double>("--follower-speed");
  std::vector<std::string> camera_strs = program.get<std::vector<std::string>>("--camera");
  int start_episode = program.get<int>("--start-episode");
  std::string task_name = program.get<std::string>("--task-name");
  int max_time_steps = program.get<int>("--max-time-steps");
  int frequency = program.get<int>("--frequency");
  std::vector<double> start_joint_pos_left = program.get<std::vector<double>>("--start-joint-pos-left");
  std::vector<double> start_joint_pos_right = program.get<std::vector<double>>("--start-joint-pos-right");

  std::size_t camera_num = camera_strs.size();
  std::vector<std::shared_ptr<WebCam>> cameras(camera_num);
  bool running = true;
  std::shared_mutex mutex_;
  std::vector<std::thread> threads;
  std::vector<std::thread> camera_threads;

  uint8_t cnt = 0;
  std::vector<std::thread> camera_threads_init;
  for (auto device : camera_strs) {
    camera_threads_init.emplace_back(std::thread(
        [&](uint8_t cnt, int device) {
          cameras[cnt] = std::make_shared<WebCam>(device);
          print("Camera " + std::to_string(cnt) + " brought up", 2, 0);
        },
        cnt, std::stoi(device)));
    cnt++;
  }
  for (auto&& ct : camera_threads_init) ct.join();

  assert(start_joint_pos_left.size() == 7 && "The size of start_joint_pos should be 7.");
  Joints<6> joint_arm_left;
  for (int i = 0; i < 6; i++) joint_arm_left[i] = start_joint_pos_left[i];
  Joints<6> joint_arm_right;
  for (int i = 0; i < 6; i++) joint_arm_right[i] = start_joint_pos_right[i];

  auto leader_left =
      std::make_unique<arm::Robot<6>>(urdf_path, master_can_left, direction, master_speed, master_end_mode);
  auto follower_left =
      std::make_unique<arm::Robot<6>>(urdf_path, node_can_left, direction, follower_speed, follower_end_mode);
  auto leader_right =
      std::make_unique<arm::Robot<6>>(urdf_path, master_can_right, direction, master_speed, master_end_mode);
  auto follower_right =
      std::make_unique<arm::Robot<6>>(urdf_path, node_can_right, direction, follower_speed, follower_end_mode);

  threads.emplace_back(std::thread([&]() {
    while (true) {
      {
        std::lock_guard<std::shared_mutex> lock(mutex_);
        if (!running) break;
      }
      follower_left->set_target_joint_q(leader_left->get_current_joint_q(), false);
      follower_left->set_target_end(leader_left->get_current_end());
      follower_right->set_target_joint_q(leader_right->get_current_joint_q(), false);
      follower_right->set_target_end(leader_right->get_current_end());
      std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
  }));

  std::vector<cv::Mat> images;
  images.resize(camera_num);
  for (int i = 0; i < camera_num; i++)
    camera_threads.emplace_back(std::thread(
        [&](std::shared_ptr<WebCam> cam, int i) {
          while (true) {
            if (!running) break;
            images[i] = cam->get_frame();
          }
        },
        cameras[i], i));

  camera_threads.emplace_back(std::thread([&]() {
    while (true) {
      if (!running) break;
      bool flag = false;
      for (int i = 0; i < camera_num; i++)
        if (images[i].empty()) {
          flag = true;
          break;
        }
      if (flag) continue;
      cv::Mat merged = mergeRGBHorizontally(images);
      if (!merged.empty()) {
        cv::imshow("Concat", merged);
        cv::waitKey(1);
      }
    }
  }));

  // record data
  std::vector<time_t> time_records_;
  std::vector<double> endt_q_records_left_, endt_v_records_left_, endf_q_records_left_, endf_v_records_left_;
  std::vector<Joints<6>> qt_records_left_, vt_records_left_, tt_records_left_, qf_records_left_, vf_records_left_,
      tf_records_left_;
  std::vector<Frame> eef_posef_records_left_, eef_poset_records_left_;
  std::vector<double> endt_q_records_right_, endt_v_records_right_, endf_q_records_right_, endf_v_records_right_;
  std::vector<Joints<6>> qt_records_right_, vt_records_right_, tt_records_right_, qf_records_right_, vf_records_right_,
      tf_records_right_;
  std::vector<Frame> eef_posef_records_right_, eef_poset_records_right_;
  std::vector<std::vector<cv::Mat>> images_records_(camera_num);
  bool recording = false;
  threads.emplace_back(std::thread([&]() {
    auto sleep_time = std::chrono::microseconds(1000000 / frequency);
    auto start_time = std::chrono::system_clock::now();
    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    while (true) {
      start_time = std::chrono::system_clock::now();
      if (!running) break;
      if (!recording) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        print("", 0, 0);
        continue;
      } else {
        if (qt_records_left_.size() < max_time_steps || qt_records_right_.size() < max_time_steps) {
          print("Recorded: " + std::to_string(qt_records_left_.size()), 0, 0);
          qt_records_left_.emplace_back(leader_left->get_current_joint_q());
          vt_records_left_.emplace_back(leader_left->get_current_joint_v());
          tt_records_left_.emplace_back(leader_left->get_current_joint_t());
          endt_q_records_left_.emplace_back(leader_left->get_current_end());
          eef_poset_records_left_.emplace_back(leader_left->get_current_pose());
          qf_records_left_.emplace_back(follower_left->get_current_joint_q());
          vf_records_left_.emplace_back(follower_left->get_current_joint_v());
          tf_records_left_.emplace_back(follower_left->get_current_joint_t());
          endf_q_records_left_.emplace_back(follower_left->get_current_end());
          eef_posef_records_left_.emplace_back(follower_left->get_current_pose());

          qt_records_right_.emplace_back(leader_right->get_current_joint_q());
          vt_records_right_.emplace_back(leader_right->get_current_joint_v());
          tt_records_right_.emplace_back(leader_right->get_current_joint_t());
          endt_q_records_right_.emplace_back(leader_right->get_current_end());
          eef_poset_records_right_.emplace_back(leader_right->get_current_pose());
          qf_records_right_.emplace_back(follower_right->get_current_joint_q());
          vf_records_right_.emplace_back(follower_right->get_current_joint_v());
          tf_records_right_.emplace_back(follower_right->get_current_joint_t());
          endf_q_records_right_.emplace_back(follower_right->get_current_end());
          eef_posef_records_right_.emplace_back(follower_right->get_current_pose());
          time_records_.emplace_back(get_timestamp());
          for (std::size_t i = 0; i < camera_num; i++) images_records_[i].emplace_back(images[i].clone());
        } else {
          print("Records full, please stop recording.", 0, 0);
        }
        end_time = std::chrono::system_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        if (duration < sleep_time)
          std::this_thread::sleep_for(sleep_time - duration);
        else
          print("Recording frequency too high!", 4, 0);
      }
    }
  }));

  // clear data
  auto clear_data = [&]() {
    qt_records_left_.clear();
    vt_records_left_.clear();
    tt_records_left_.clear();
    endt_q_records_left_.clear();
    endt_v_records_left_.clear();
    eef_poset_records_left_.clear();
    qf_records_left_.clear();
    vf_records_left_.clear();
    tf_records_left_.clear();
    endf_q_records_left_.clear();
    endf_v_records_left_.clear();
    eef_posef_records_left_.clear();

    qt_records_right_.clear();
    vt_records_right_.clear();
    tt_records_right_.clear();
    endt_q_records_right_.clear();
    endt_v_records_right_.clear();
    eef_poset_records_right_.clear();
    qf_records_right_.clear();
    vf_records_right_.clear();
    tf_records_right_.clear();
    endf_q_records_right_.clear();
    endf_v_records_right_.clear();
    eef_posef_records_right_.clear();

    time_records_.clear();
    for (std::size_t i = 0; i < camera_num; i++) images_records_[i].clear();
  };

  // init terminal
  initscr();             // Initialize ncurses
  raw();                 // Disable line buffering
  keypad(stdscr, TRUE);  // Enable special keys
  noecho();              // Disable echoing of characters
  timeout(2);

  // main: keyborad control
  int series_count = start_episode;  // 从指定的序号开始保存数据
  bool gravity_compensation_flag = false;
  while (1) {
    int ch = get_key();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    switch (ch) {
      case ' ':  // 空格按下开始记录，再按下结束记录保存数据
        if (!recording) {
          recording = true;
          gravity_compensation_flag = true;
          leader_left->manual_mode();
          leader_right->manual_mode();
          print("", 1, 0);
          print("", 2, 0);
        } else {
          if (qt_records_left_.size() < max_time_steps || qt_records_right_.size() < max_time_steps) break;
          recording = false;
          leader_left->online_mode();
          leader_right->online_mode();
          gravity_compensation_flag = false;
          std::string target_dir = "demonstrations/raw/" + task_name + "/" + std::to_string(series_count);
          createDirIfNotExists(target_dir);
          print("Start saving data to json...", 1, 0);

          nlohmann::json data;
          data["count_left"] = qt_records_left_.size();
          data["count_right"] = qt_records_right_.size();
          data["/observations/pos_t_left"] = qt_records_left_;
          data["/observations/vel_t_left"] = vt_records_left_;
          data["/observations/eff_t_left"] = tt_records_left_;
          data["/observations/endpos_t_left"] = endt_q_records_left_;
          data["/observations/eef_pose_t_left"] = eef_poset_records_left_;
          data["/observations/pos_f_left"] = qf_records_left_;
          data["/observations/vel_f_left"] = vf_records_left_;
          data["/observations/eff_f_left"] = tf_records_left_;
          data["/observations/endpos_f_left"] = endf_q_records_left_;
          data["/observations/eef_pose_f_left"] = eef_posef_records_left_;

          data["/observations/pos_t_right"] = qt_records_right_;
          data["/observations/vel_t_right"] = vt_records_right_;
          data["/observations/eff_t_right"] = tt_records_right_;
          data["/observations/endpos_t_right"] = endt_q_records_right_;
          data["/observations/eef_pose_t_right"] = eef_poset_records_right_;
          data["/observations/pos_f_right"] = qf_records_right_;
          data["/observations/vel_f_right"] = vf_records_right_;
          data["/observations/eff_f_right"] = tf_records_right_;
          data["/observations/endpos_f_right"] = endf_q_records_right_;
          data["/observations/eef_pose_f_right"] = eef_posef_records_right_;
          data["/observations/time"] = time_records_;
          std::ofstream f(target_dir + "/records.json");
          f << data.dump(2);
          f.close();

          print("Start saving images to video...", 1, 0);
          std::vector<std::thread> camera_saving_threads;
          for (std::size_t i = 0; i < camera_num; i++)
            camera_saving_threads.emplace_back(std::thread(
                [&](u_int8_t index) {
                  std::string video_path = target_dir + "/" + std::to_string(index) + ".avi";
                  auto image_size = cv::Size(images[index].size().width, images[index].size().height);
                  cv::VideoWriter video(video_path, cv::VideoWriter::fourcc('F', 'F', 'V', '1'), frequency, image_size);
                  for (std::size_t j = 0; j < images_records_[index].size(); j++)
                    video.write(images_records_[index][j]);
                  video.release();
                },
                i));
          for (auto&& ctd : camera_saving_threads) ctd.join();
          series_count++;
          clear_data();
          leader_left->set_target_joint_q(joint_arm_left);
          leader_left->set_target_end(start_joint_pos_left[6]);
          leader_right->set_target_joint_q(joint_arm_right);
          leader_right->set_target_end(start_joint_pos_right[6]);
          print("All data saved.", 1, 0);
        }
        break;
      case '[':
        leader_left->set_target_end(0);
        leader_right->set_target_end(0);
        break;
      case ']':
        leader_left->set_target_end(1);
        leader_right->set_target_end(1);
        break;
      case 'g':
        if (!gravity_compensation_flag) {
          leader_left->manual_mode();
          leader_right->manual_mode();
          gravity_compensation_flag = true;
        } else {
          leader_left->online_mode();
          leader_right->online_mode();
          gravity_compensation_flag = false;
        }
        break;
      case 'q':
        recording = false;
        leader_left->online_mode();
        leader_right->online_mode();
        gravity_compensation_flag = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        clear_data();
        break;
      case '0':  // return to starting point
        leader_left->set_target_joint_q(joint_arm_left);
        leader_left->set_target_end(start_joint_pos_left[6]);
        leader_right->set_target_joint_q(joint_arm_right);
        leader_right->set_target_end(start_joint_pos_right[6]);
        gravity_compensation_flag = false;
        break;
      case 'p':  // 打印当前位置
        print("Current joint positions of left: " + std::to_string(leader_left->get_current_joint_q()[0]) + " " +
                  std::to_string(leader_left->get_current_joint_q()[1]) + " " +
                  std::to_string(leader_left->get_current_joint_q()[2]) + " " +
                  std::to_string(leader_left->get_current_joint_q()[3]) + " " +
                  std::to_string(leader_left->get_current_joint_q()[4]) + " " +
                  std::to_string(leader_left->get_current_joint_q()[5]) + " " +
                  std::to_string(leader_left->get_current_end()),
              2, 0);
        print("Current joint positions of right: " + std::to_string(leader_right->get_current_joint_q()[0]) + " " +
                  std::to_string(leader_right->get_current_joint_q()[1]) + " " +
                  std::to_string(leader_right->get_current_joint_q()[2]) + " " +
                  std::to_string(leader_right->get_current_joint_q()[3]) + " " +
                  std::to_string(leader_right->get_current_joint_q()[4]) + " " +
                  std::to_string(leader_right->get_current_joint_q()[5]) + " " +
                  std::to_string(leader_right->get_current_end()),
              3, 0);
        print("Current eef pose of left: " + std::to_string(leader_left->get_current_pose().first[0]) + " " +
                  std::to_string(leader_left->get_current_pose().first[1]) + " " +
                  std::to_string(leader_left->get_current_pose().first[2]) + " " +
                  std::to_string(leader_left->get_current_pose().second[0]) + " " +
                  std::to_string(leader_left->get_current_pose().second[1]) + " " +
                  std::to_string(leader_left->get_current_pose().second[2]) + " " +
                  std::to_string(leader_left->get_current_pose().second[3]),
              4, 0);
        print("Current eef pose of right: " + std::to_string(leader_right->get_current_pose().first[0]) + " " +
                  std::to_string(leader_right->get_current_pose().first[1]) + " " +
                  std::to_string(leader_right->get_current_pose().first[2]) + " " +
                  std::to_string(leader_right->get_current_pose().second[0]) + " " +
                  std::to_string(leader_right->get_current_pose().second[1]) + " " +
                  std::to_string(leader_right->get_current_pose().second[2]) + " " +
                  std::to_string(leader_right->get_current_pose().second[3]),
              5, 0);
        break;
      default:
        break;
    }
    if (ch == 'z' || ch == 3) {
      print("Exiting...", 0, 0);
      print("", 1, 0);
      print("", 2, 0);
      print("", 3, 0);
      break;
    }
  }

  {
    std::lock_guard<std::shared_mutex> lock(mutex_);
    running = false;
    recording = false;
  }

  for (auto&& i : threads) i.join();
  auto reset_leader_left = std::thread([&]() { leader_left.reset(); });
  auto reset_follower_left = std::thread([&]() { follower_left.reset(); });
  auto reset_leader_right = std::thread([&]() { leader_right.reset(); });
  auto reset_follower_right = std::thread([&]() { follower_right.reset(); });
  reset_leader_left.join();
  reset_follower_left.join();
  reset_leader_right.join();
  reset_follower_right.join();
  for (auto&& i : camera_threads) i.join();
  endwin();
  return 0;
}

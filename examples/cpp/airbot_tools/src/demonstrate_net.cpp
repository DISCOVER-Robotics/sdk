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

std::string post(const std::string& host, int port, const std::string& path, const std::string& body) {
  // 创建socket
  int sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd == -1) throw HTTPException("Failed to create socket");

  // 获取服务器地址信息
  struct addrinfo hints {
  }, *serverAddrInfo = nullptr;
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = 0;
  hints.ai_flags = AI_NUMERICSERV;

  std::ostringstream portStr;
  portStr << port;
  if (getaddrinfo(host.c_str(), portStr.str().c_str(), &hints, &serverAddrInfo) != 0) {
    close(sockfd);
    throw HTTPException("Failed to get address info");
  }

  // 连接到服务器
  if (connect(sockfd, serverAddrInfo->ai_addr, serverAddrInfo->ai_addrlen) < 0) {
    close(sockfd);
    freeaddrinfo(serverAddrInfo);
    throw HTTPException("Failed to connect to server");
  }

  // 构造POST请求的内容
  std::ostringstream request;
  request << "POST " << path << " HTTP/1.1\r\n";
  request << "Host: " << host << "\r\n";
  request << "Content-Type: application/json\r\n";
  request << "Content-Length: " << body.length() << "\r\n";
  request << "Connection: close\r\n\r\n";
  request << body;

  // 发送HTTP请求
  if (write(sockfd, request.str().c_str(), request.str().length()) < 0) {
    std::cerr << "Failed to send request" << std::endl;
    close(sockfd);
    freeaddrinfo(serverAddrInfo);
    throw HTTPException("Failed to send request");
  }

  // 接收和处理服务器的响应
  std::ostringstream response;
  char buffer[4096];
  ssize_t bytesRead;
  while ((bytesRead = read(sockfd, buffer, sizeof(buffer) - 1)) > 0) {
    buffer[bytesRead] = '\0';
    response << buffer;
  }

  // 关闭连接
  close(sockfd);
  freeaddrinfo(serverAddrInfo);

  // 返回响应内容
  return response.str();
}

void color_print(std::string s, int w = 0, int h = 0, std::string color = "black") {
  move(w, h);
  clrtoeol();
  printw(s.c_str());
  refresh();
}

int get_key() {
  int ch = getch();
  return ch;
}

auto split_host_port(const std::string& host_port) {
  auto pos = host_port.find(':');
  if (pos == std::string::npos) {
    return std::make_pair(host_port, 80);
  }
  return std::make_pair(host_port.substr(0, pos), std::stoi(host_port.substr(pos + 1)));
}

std::string ConvertDoubleToJson(const vector<double>& vec, double end) {
  std::ostringstream oss;
  oss << "{\"data\": [";
  for (size_t i = 0; i < vec.size(); ++i) {
    oss << vec[i];
    if (i != vec.size() - 1) {
      oss << ",";
    }
  }
  oss << "], \"end\": " << end << "}";
  return oss.str();
}

std::string ConvertQVToJson(const vector<vector<double>>& vecs, double end) {
  std::ostringstream oss;
  oss << "{\"data\": [";
  for (size_t i = 0; i < vecs[0].size(); ++i) {
    oss << vecs[0][i];
    if (i != vecs[0].size() - 1) {
      oss << ",";
    }
  }
  oss << "],";

  oss << "\"data2\": [";
  for (size_t i = 0; i < vecs[1].size(); ++i) {
    oss << vecs[1][i];
    if (i != vecs[1].size() - 1) {
      oss << ",";
    }
  }
  oss << "],";

  oss << "\"end\": " << end << "}";
  return oss.str();
}

std::string post_init(const std::string& host, int port, int retry = 3) {
  std::string path = "/init";
  std::string ret;
  for (int i = 0; i < retry; i++) {
    try {
      ret = post(host, port, path, "");
    } catch (const HTTPException& e) {
      std::cerr << e.what() << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return ret;
}

std::string post_finish(const std::string& host, int port, int retry = 3) {
  std::string path = "/erase";
  std::string ret;
  for (int i = 0; i < retry; i++) {
    try {
      ret = post(host, port, path, "");
    } catch (const HTTPException& e) {
      std::cerr << e.what() << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return ret;
}

std::string post_end(arm::Robot* robot, const std::string& host, int port, int retry = 3) {
  std::string path = "/set_target_end";
  std::string ret;
  auto json_data = ConvertQVToJson(robot->get_current_joint_q_v(), robot->get_current_end());
  for (int i = 0; i < retry; i++) {
    try {
      ret = post(host, port, path, json_data);
    } catch (const HTTPException& e) {
      std::cerr << e.what() << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return ret;
}

int main(int argc, char** argv) {
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
  program.add_argument("-r", "--remote")
      .default_value<std::vector<std::string>>({})
      .nargs(argparse::nargs_pattern::any)
      .help(
          "The remote host and port of following arm. E.g., 192.168.1.1:8888. "
          "Can use multiple times (multiple remote "
          "following arms)");
  program.add_argument("-c", "--camera")
      .scan<'i', int>()
      .default_value(-1)
      .help("The camera device index attached to the master arm");
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
  program.add_argument("-f", "--follower-end-mode")
      .default_value("gripper")
      .choices("teacher", "gripper", "yinshi", "newteacher", "none")
      .help(
          "The mode of the follower arm end effector. Available choices: \n"
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
  program.add_argument("--follower-mode")
      .default_value("q")
      .choices("q", "mirror")
      .help(
          "The mode of following. Available choices: \n"
          "\"q\": follow only motor pos \n"
          "\"mirror\": mirror following.");
  program.add_argument("--master-speed")
      .scan<'g', double>()
      .default_value(1.)
      .help("The joint speed of the master arm in ratio of PI.");
  program.add_argument("--follower-speed")
      .scan<'g', double>()
      .default_value(2.)
      .help("The joint speed of the follower arm in ratio of PI.");
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

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }
  std::string urdf_path = program.get<std::string>("--urdf");
  std::string master_can = program.get<std::string>("--master");
  std::vector<std::string> node_cans = program.get<std::vector<std::string>>("--node");
  std::vector<std::string> remote_hosts = program.get<std::vector<std::string>>("--remote");
  std::string trajectory_path = program.get<std::string>("--trajectory");
  std::string master_end_mode = program.get<std::string>("--master-end-mode");
  std::string follower_end_mode = program.get<std::string>("--follower-end-mode");
  std::string follower_mode = program.get<std::string>("--follower-mode");
  std::string direction = program.get<std::string>("--direction");
  double master_speed = program.get<double>("--master-speed");
  double follower_speed = program.get<double>("--follower-speed");
  bool constrained = program.get<bool>("--constrained");
  bool real_time = program.get<bool>("--real-time");
  int camera = program.get<int>("--camera");
  std::optional<uint16_t> video_device_;
  if (camera != -1) {
    video_device_ = camera;
  } else {
    video_device_ = std::nullopt;
  }
  if (urdf_path == "") {
    if (master_end_mode == "none")
      urdf_path = URDF_INSTALL_PATH + "airbot_play_v2_1.urdf";
    else
      urdf_path = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf";
  }

  bool running = true;
  std::shared_mutex mutex_;
  std::vector<std::thread> threads;
  std::vector<std::vector<double>> manual_waypoints_;
  std::vector<double> manual_ends_;
  std::vector<int64_t> manual_times_;

  // Synchronization
  arm::Robot* robot;
  try {
    robot = new arm::Robot(std::make_unique<arm::AnalyticFKSolver>(urdf_path),
                           std::make_unique<arm::AnalyticIKSolver>(urdf_path),
                           std::make_unique<arm::ChainIDSolver>(urdf_path, direction), master_can.c_str(), master_speed,
                           master_end_mode, constrained, real_time);
  } catch (const std::runtime_error& e) {
    std::cerr << e.what() << '\n';
    endwin();
    return 1;
  }

  robot->record_load(trajectory_path);

  std::vector<arm::Robot*> followers;
  for (auto&& i : node_cans) {
    try {
      auto follower = new arm::Robot(
          std::make_unique<arm::AnalyticFKSolver>(urdf_path), std::make_unique<arm::AnalyticIKSolver>(urdf_path),
          std::make_unique<arm::ChainIDSolver>(urdf_path, direction), i.c_str(), follower_speed, follower_end_mode);
      followers.push_back(follower);
      threads.push_back(std::thread([&]() {
        while (true) {
          {
            std::lock_guard<std::shared_mutex> lock(mutex_);
            if (!running) break;
          }
          if (follower_mode == "mirror") {
            auto current_pose = robot->get_current_pose();
            auto current_translation = current_pose[0];
            auto current_rotation = current_pose[1];

            current_translation[1] = -current_translation[1];
            auto current_quaternion = vector2rotation(current_rotation);
            KDL::Vector current_axis_angle = current_quaternion.GetRot();
            double current_angle =
                sqrt(current_axis_angle[0] * current_axis_angle[0] + current_axis_angle[1] * current_axis_angle[1] +
                     current_axis_angle[2] * current_axis_angle[2]);
            KDL::Vector current_axis = current_axis_angle / current_angle;
            current_axis[1] = -current_axis[1];
            current_quaternion = KDL::Rotation::Rot(current_axis, -current_angle);
            std::vector<double> current_vector_quaternion = rotation2vector(current_quaternion);
            follower->set_target_pose(current_translation, current_vector_quaternion);
          } else if (follower_mode == "q") {
            follower->set_target_joint_q(robot->get_current_joint_q());
          } else if (follower_mode == "qv") {
            follower->set_target_joint_q_v(robot->get_current_joint_q_v());
          }
          follower->set_target_end(robot->get_current_end());

          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }));
    } catch (const std::runtime_error& e) {
      std::cerr << e.what() << '\n';
      endwin();
      {
        std::lock_guard<std::shared_mutex> lock(mutex_);
        running = false;
      }
      for (auto&& i : threads) i.join();
      for (auto&& i : followers) delete i;
      delete robot;
      return 1;
    }
  }
  for (auto&& i : remote_hosts) {
    auto [host_, port_] = split_host_port(i);
    post_init(host_, port_);
    threads.push_back(std::thread(
        [&](std::string host, int port) {
          while (true) {
            {
              std::lock_guard<std::shared_mutex> lock(mutex_);
              if (!running) break;
            }
            post_end(robot, host, port);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
          }
        },
        host_, port_));
  }

  // init terminal
  filter();
  initscr();             // Initialize ncurses
  raw();                 // Disable line buffering
  keypad(stdscr, TRUE);  // Enable special keys
  noecho();              // Disable echoing of characters
  timeout(2);

  // Manipulation of Master arm
  auto from_base = true;
  auto step = 0.01;
  auto angle_step = 0.1;
  auto gripper_state = false;
  double x, y, z, w;
  while (1) {
    int ch = get_key();
    uint32_t end_snap_signal = robot->get_end_snap_signal();
    uint32_t base_snap_signal = robot->get_base_snap_signal();
    if (end_snap_signal || base_snap_signal) {
      robot->end_snap_signal_handle(end_snap_signal);
      robot->base_snap_signal_handle(base_snap_signal);
    } else {
      switch (ch) {
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
          if (from_base) {
            robot->add_target_translation({step, 0, 0});
          } else {
            robot->add_target_relative_translation({step, 0, 0});
          }
          break;
        case 's':
          if (from_base) {
            robot->add_target_translation({-step, 0, 0});
          } else {
            robot->add_target_relative_translation({-step, 0, 0});
          }
          break;
        case 'a':
          if (from_base) {
            robot->add_target_translation({0, step, 0});
          } else {
            robot->add_target_relative_translation({0, step, 0});
          }
          break;
        case 'd':
          if (from_base) {
            robot->add_target_translation({0, -step, 0});
          } else {
            robot->add_target_relative_translation({0, -step, 0});
          }
          break;
        case 'q':
          if (from_base) {
            robot->add_target_translation({0, 0, step});
          } else {
            robot->add_target_relative_translation({0, 0, step});
          }
          break;
        case 'e':
          if (from_base) {
            robot->add_target_translation({0, 0, -step});
          } else {
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
      if (ch == 'z' || ch == 3) break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  endwin();

  {
    std::lock_guard<std::shared_mutex> lock(mutex_);
    running = false;
  }

  for (auto&& i : remote_hosts) {
    auto [host, port] = split_host_port(i);
    post_finish(host, port);
  }

  for (auto&& i : threads) i.join();
  for (auto&& i : followers) delete i;
  delete robot;

  return 0;
}

#include <airbot/airbot.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

#include "argparse/argparse.hpp"
#include "camera/webcam.hpp"
const std::string red("\033[0;31m");
const std::string green("\033[0;32m");
const std::string yellow("\033[0;33m");
const std::string blue("\033[0;34m");
const std::string cyan("\033[0;36m");
const std::string reset("\033[0m");

constexpr std::array<double, 3> MOTOR_DOWN_ANGLE = {0., 0.1922, -0.1376};
constexpr std::array<double, 3> RANGE = {0.1, 0.1, 0.1};

constexpr bool in_range(const std::array<double, 3> input) {
  return (MOTOR_DOWN_ANGLE[0] - RANGE[0] < input[0] && input[0] < MOTOR_DOWN_ANGLE[0] + RANGE[0] &&
          MOTOR_DOWN_ANGLE[1] - RANGE[1] < input[1] && input[1] < MOTOR_DOWN_ANGLE[1] + RANGE[1] &&
          MOTOR_DOWN_ANGLE[2] - RANGE[2] < input[2] && input[2] < MOTOR_DOWN_ANGLE[2] + RANGE[2]);
}

int main(int argc, char **argv) {
  argparse::ArgumentParser program("tool_set_zero", AIRBOT_VERSION);
  program.add_description(
      "This is a tool to set zero points for the motors of AIRBOT Play. This "
      "tool should only be used during "
      "manufacturing and maintenance. It is not recommended to use this tool "
      "in normal operation\n"
      "If an end effector is attached, it is important to keep the end "
      "effector in zero position during the process.");
  program.add_argument("-m", "--master")
      .required()
      .default_value("can0")
      .help("Can device interface of the master arm. Default: can0");
  program.add_argument("-e", "--master-end-mode")
      .default_value("newteacher")
      .choices("teacher", "gripper", "yinshi", "newteacher", "none")
      .help(
          "The mode of the master arm end effector. Available choices: \n"
          "\"teacher\": The demonstrator equipped with Damiao motor \n"
          "\"gripper\": The gripper equipped with Damiao motor \n"
          "\"yinshi\": The Yinshi two-finger gripper \n"
          "\"newteacher\": The demonstrator equipped with self-developed "
          "motor \n"
          "\"none\": The arm is not equipped with end effector.");
  program.add_argument("-c", "--camera")
      .scan<'i', int>()
      .default_value(-1)
      .help("The camera device index attached to the master arm");

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }
  // setup logger
  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      (std::string("logs/tool_set_zero_logs") + arm::get_timestring() + ".log").c_str(), 1024 * 1024, 10, false));
  std::shared_ptr<spdlog::logger> logger = arm::setup_logger(sinks);
  spdlog::flush_every(std::chrono::seconds(1));
  logger->set_level(spdlog::level::info);

  std::string interface = program.get<std::string>("--master");
  std::string gripper_type = program.get<std::string>("--master-end-mode");
  int camera = program.get<int>("--camera");
  system((std::string("ip link set ") + std::string(interface) + std::string(" up type can bitrate 1000000")).c_str());
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  bool running = true;
  std::thread camera_thread;
  std::shared_ptr<WebCam> camera_obj;
  if (camera >= 0) {
    camera_obj = std::make_shared<WebCam>(camera);
    camera_thread = std::thread([&](std::shared_ptr<WebCam> cam) { cam->display_frame(running); }, camera_obj);
  }
  auto release_brake = 1;
  std::vector<std::unique_ptr<arm::MotorDriver>> motor_driver_;
  for (uint8_t i = 0; i < 3; i++) {
    motor_driver_.push_back(arm::MotorDriver::MotorCreate(i + 1, interface.c_str(), logger, "OD"));
    motor_driver_[i]->MotorInit();
    motor_driver_[i]->set_motor_control_mode(arm::MotorDriver::MIT);
  }
  for (uint8_t i = 3; i < 6; i++) {
    motor_driver_.push_back(arm::MotorDriver::MotorCreate(i + 1, interface.c_str(), logger, "DM"));
    motor_driver_[i]->MotorInit();
    motor_driver_[i]->set_motor_control_mode(arm::MotorDriver::MIT);
  }
  if (gripper_type != "none") {
    motor_driver_.push_back(arm::MotorDriver::MotorCreate(7, interface.c_str(), logger, "OD"));
    motor_driver_[6]->MotorInit();
    motor_driver_[6]->set_motor_control_mode(arm::MotorDriver::MIT);
  }
  auto thread_brake = std::thread([&]() {
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (release_brake == 0)
        continue;
      else if (release_brake == 2)
        break;
      for (int i = 0; i < 3; i++) {
        motor_driver_[i]->MotorMitModeCmd(0, 0, 0, 2, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
      }
      for (int i = 3; i < motor_driver_.size(); i++) {
        motor_driver_[i]->MotorMitModeCmd(0, 0, 0, 1, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
      }
    }
  });

  std::cout << blue << "放入标零工具，将机械臂牵引到零点后，按下回车键" << reset << std::endl;
  std::string tmp;
  getline(std::cin, tmp);

  release_brake = 0;
  for (auto &&i : motor_driver_) {
    i->MotorSetZero();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  for (auto &&i : motor_driver_) {
    i->MotorWriteFlash();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  std::cout << green << "标零完成。 " << reset << std::endl;
  release_brake = 1;

  std::cout << blue << "移去标零工具，按下回车键" << reset << std::endl;
  getline(std::cin, tmp);
  release_brake = 2;
  thread_brake.join();

  for (auto &&i : motor_driver_) {
    i->MotorInit();  // This is to init DM motors after timeout due to
                     // thread_brake.join()
    i->set_motor_control_mode(arm::MotorDriver::POS);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  while (std::abs(motor_driver_[0]->get_motor_pos()) > 0.1 || std::abs(motor_driver_[1]->get_motor_pos()) > 0.1 ||
         std::abs(motor_driver_[2]->get_motor_pos()) > 0.1 || std::abs(motor_driver_[3]->get_motor_pos()) > 0.1 ||
         std::abs(motor_driver_[4]->get_motor_pos()) > 0.1 || std::abs(motor_driver_[5]->get_motor_pos()) > 0.1) {
    for (auto &&i : motor_driver_) {
      i->MotorPosModeCmd(0, 0.5);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  for (auto &&i : motor_driver_) i->MotorDeInit();

  running = false;
  if (camera_thread.joinable()) camera_thread.join();
  return 0;
}

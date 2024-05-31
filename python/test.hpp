#ifndef COMMAND_BASE_HPP
#define COMMAND_BASE_HPP

#include <Eigen/Dense>
#include <atomic>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include "httplib.h"
#undef _res

#include "airbot/command/command_types.hpp"
#include "airbot/modules/boards/interface_board_base.hpp"
#include "airbot/modules/boards/interface_board_end.hpp"
#include "airbot/modules/controller/fk.hpp"
#include "airbot/modules/controller/fk_analytic.hpp"
#include "airbot/modules/controller/id.hpp"
#include "airbot/modules/controller/id_chain_rne.hpp"
#include "airbot/modules/controller/ik.hpp"
#include "airbot/modules/controller/ik_analytic.hpp"
#include "airbot/modules/controller/ikv_chain.hpp"
#include "airbot/modules/motors/motor_driver.hpp"
#include "airbot/utils.hpp"

#define MAGIC_DELAY 200  // This MAGIC_DELAY should be removed in the future

using arm::MotorDriver;
using std::vector;
using SnapMode = arm::BoardDriver::snap_mode_e;
namespace fs = std::filesystem;
constexpr const double DEFAULT_VEL = 0.2;
constexpr const double LOOK_AHEAD = 0;
constexpr const double QUEUE_SIZE = 10000;
constexpr const uint32_t BLOCK_SPIN_TIME = 10;  // 10 milliseconds
constexpr const double BLOCK_THRESHOLD = 0.005;
constexpr const uint32_t BLOCK_TIMEOUT = 30000;  // 30 second
constexpr const double MAX_VEL_LIMIT = M_PI;
constexpr const double RECOVERY_SPD_RATIO = 0.25;

class KibanaLogger {
 private:
  bool inited_;
  std::string url_;
  std::string topic_name_;
  std::unique_ptr<httplib::Client> client_;

 public:
  KibanaLogger(std::string url = "http://192.168.112.156:9200", std::string topic_name = "debug")
      : url_(url), topic_name_(topic_name), inited_(false) {}

  template <std::size_t DOF>
  void push_remote_log_once(const LoggingData<DOF>& data) {
    if (!inited_) {
      client_ = std::make_unique<httplib::Client>(url_.c_str());
      nlohmann::json document;
      document["mappings"]["properties"]["timestamp"]["type"] = "date";
      document["mappings"]["properties"]["feedback"]["properties"]["timestamp"]["type"] = "date";
      client_->Put(std::string("/") + topic_name_, document.dump(), "application/json");
      inited_ = true;
    }
    nlohmann::json document;

    auto time_stamp = data.fb_data.time_stamp / 1000;
    auto current_state = data.fb_data.current_state;
    auto postion = data.fb_data.current_joint_q;
    auto velocity = data.fb_data.current_joint_v;
    auto torque = data.fb_data.current_joint_t;
    auto temperature = data.fb_data.current_joint_temp;
    auto err = data.fb_data.current_joint_err;
    auto resp_cnt = data.fb_data.response_cnt;
    auto end_pose = data.fb_data.current_pose;
    auto current_end = data.fb_data.current_end;

    auto target_state = data.cmd_data.cmd_state;
    auto target_position = data.cmd_data.target_joint_q;
    auto target_plan_position = data.cmd_data.plan_target_joint_q;
    auto target_velocity = data.cmd_data.target_joint_v;
    auto target_torque = data.cmd_data.target_joint_t;
    auto target_kp = data.cmd_data.target_joint_kp;
    auto target_kd = data.cmd_data.target_joint_kd;
    auto target_replay = data.cmd_data.replay_request;
    auto target_recover = data.cmd_data.recover_request;
    auto target_end = data.cmd_data.target_end;

    document["feedback"]["timestamp"] = time_stamp;

    for (int i = 0; i < data.cmd_data.target_joint_q.size(); i++) {
      document["feedback"][std::to_string(i)]["position"] = postion[i];
      document["feedback"][std::to_string(i)]["velocity"] = velocity[i];
      document["feedback"][std::to_string(i)]["torque"] = torque[i];
      document["feedback"][std::to_string(i)]["temperature"] = temperature[i];
      document["feedback"][std::to_string(i)]["error_id"] = err[i];
      document["feedback"][std::to_string(i)]["response_cnt"] = resp_cnt[i];
    }
    document["feedback"]["end_pose"]["x"] = end_pose.first[0];
    document["feedback"]["end_pose"]["y"] = end_pose.first[1];
    document["feedback"]["end_pose"]["z"] = end_pose.first[2];
    document["feedback"]["end_pose"]["qx"] = end_pose.second[0];
    document["feedback"]["end_pose"]["qy"] = end_pose.second[1];
    document["feedback"]["end_pose"]["qz"] = end_pose.second[2];
    document["feedback"]["end_pose"]["qw"] = end_pose.second[3];

    document["feedback"]["current_end"] = current_end;
    document["feedback"]["current_state"] = current_state;

    for (int i = 0; i < data.cmd_data.target_joint_q.size(); i++) {
      document["command"][std::to_string(i)]["position"] = target_position[i];
      document["command"][std::to_string(i)]["plan_position"] = target_plan_position[i];
      document["command"][std::to_string(i)]["velocity"] = target_velocity[i];
      document["command"][std::to_string(i)]["torque"] = target_torque[i];
      document["command"][std::to_string(i)]["kp"] = target_kp[i];
      document["command"][std::to_string(i)]["kd"] = target_kd[i];
    }
    document["command"]["target_state"] = target_state;
    document["command"]["replay_request"] = int(target_replay);
    document["command"]["recover_request"] = int(target_recover);
    document["command"]["target_end"] = target_end;
    // document["timestamp"] =
    //     std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
    //         .count();
    document["timestamp"] = time_stamp;
    document["type"] = "debug";
    document["sn"] = data.sn_code;

    client_->Post(std::string("/") + topic_name_ + "/_doc", document.dump(), "application/json");
  }
};

namespace arm {
inline array<double, 4> calc(double s_q, double s_v, double t_q, double t_v) {
  return {s_q, s_v, 3 * (t_q - s_q) - 2 * s_v - 1 * t_v, -2 * (t_q - s_q) + s_v + t_v};
}

template <std::size_t DOF>
inline array<array<double, 4>, DOF> calc_plan(const Joints<DOF>& current_q, const Joints<DOF>& current_v,
                                              const Joints<DOF>& target_q, const Joints<DOF>& target_v) {
  array<array<double, 4>, DOF> plan_params;
  for (int i = 0; i < plan_params.size(); i++)
    plan_params[i] = calc(current_q[i], current_v[i] * 1, target_q[i], target_v[i]);
  return plan_params;
}

template <std::size_t DOF>
inline Joints<DOF> plan_infer(const array<array<double, 4>, DOF>& plan_params, double t) {
  auto ret = Joints<DOF>();
  for (int i = 0; i < plan_params.size(); i++)
    ret[i] = plan_params[i][0] + plan_params[i][1] * t + plan_params[i][2] * t * t + plan_params[i][3] * t * t * t;
  return ret;
}

template <std::size_t DOF>
class Robot {
  static std::unordered_map<std::string, std::pair<double, double>> end_limits;
  inline static double e2i(const double& end, const std::string& mode) {
    auto limits = end_limits[mode];
    return (end - limits.first) / (limits.second - limits.first);
  }

  inline static double i2e(const double& i, const std::string& mode) {
    auto limits = end_limits[mode];
    return i * (limits.second - limits.first) + limits.first;
  }

 private:
  /**
   * Robot status
   */
  std::atomic<ArmMode> arm_mode_;
  std::atomic<bool> is_running_, use_planning_, joint_safe_detect_, is_init_;
  std::atomic<uint32_t> logging_freq_;
  RobotCmdData<DOF> robot_cmd_data_;
  RobotFeedbackData<DOF> robot_fb_data_;

  std::atomic<uint32_t> counter_;
  std::atomic<time_t> reported_time_;
  std::atomic<uint64_t> replay_index_;
  RobotPlanData<DOF> robot_plan_data_;
  RobotRecordData<DOF> recorded_data_, replay_data_;
  time_t last_update_time_;
  time_t replay_inplace_time_;

  /**
   * Robot parameters
   */
  std::string end_effector_type_;
  Joints<DOF> joint_vel_limit_;

  /**
   * Logging
   */
  std::shared_ptr<spdlog::logger> logger_;
  Queue<DOF> logging_queue_;
  KibanaLogger kibana_logger_;
  std::atomic<uint64_t> last_logging_time_;

  /**
   * Robot modules
   */
  std::unique_ptr<FKSolver<DOF>> fk_solver_;
  std::unique_ptr<IKSolver<DOF>> ik_solver_;
  std::unique_ptr<IKVSolver<DOF>> ikv_solver_;
  std::unique_ptr<IDSolver<DOF>> id_solver_;
  array<std::unique_ptr<MotorDriver>, DOF> motor_driver_;
  std::unique_ptr<MotorDriver> end_motor_driver_;
  std::unique_ptr<InterfaceBoardBase> interface_board_base_;
  std::unique_ptr<InterfaceBoardEnd> interface_board_end_;

  /**
   * Thread handles
   */
  std::thread main_update_thread_;
  std::thread logging_thread_;

  /**
   * Mutexes
   */
  mutable std::mutex cmd_mutex_;
  mutable std::shared_mutex fb_mutex_;
  mutable std::mutex record_mutex_;

 protected:
  bool _plan_target_joint_q(const Joints<DOF>& target_joint_q, bool use_planning = true, double vel = DEFAULT_VEL,
                            bool commit = true);

  // update whole data once
  void _update_once();

  inline void _set_state(MotorControlState target_state) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    robot_cmd_data_.cmd_state = target_state;
  }

  inline void _write_fb_data(const RobotFeedbackData<DOF>& fb_tmp_data) {
    std::unique_lock<std::shared_mutex> fb_lock(fb_mutex_);
    robot_fb_data_ = fb_tmp_data;
  }

  inline void _write_cmd_data(const RobotCmdData<DOF>& cmd_tmp_data) {
    std::unique_lock<std::mutex> cmd_lock(cmd_mutex_);
    robot_cmd_data_ = cmd_tmp_data;
  }

  // External trigger to change arm mode
  inline bool change_mode(const ArmMode& cmd_mode) {
    RobotCmdData<DOF> robot_cmd_data;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      robot_cmd_data = robot_cmd_data_;
    }
    auto arm_mode = arm_mode_.load(std::memory_order_relaxed);
    return _check_mode_change(arm_mode, cmd_mode) && _on_mode_change(arm_mode, cmd_mode, robot_cmd_data);
  }

  bool _check_mode_change(const ArmMode& current_mode, const ArmMode& cmd_mode);

  // Perform mode change. Return true if mode is changed
  // By design if mode change happens actions will be performed in the next round
  // No check is performed to see if the mode should be changed
  bool _on_mode_change(const ArmMode& current_mode, const ArmMode& cmd_mode, RobotCmdData<DOF>& robot_cmd_data);

 public:
  Robot(std::string urdf_path, std::string can_interface, std::string direction = "down", double vel = M_PI,
        std::string end_mode = "teacher", std::string forearm_type = "DM", bool factory = false);
  ~Robot();

  inline void logging(uint32_t logging_freq) {
    if (logging_freq > 100) {
      logger_->warn("Logging frequency is too high, set to 100Hz");
      logging_freq = 100;
    }
    if (logging_freq == 0) {
      logger_->info("Turning off kibana logging");
      logging_queue_ = Queue<DOF>(QUEUE_SIZE);
    } else
      logger_->info("Setting kibana logging to {}Hz", logging_freq);

    logging_freq_.store(logging_freq, std::memory_order_relaxed);
  }

  Frame get_current_pose() const {
    std::shared_lock<std::shared_mutex> lock(fb_mutex_);
    return robot_fb_data_.current_pose;
  };

  Joints<DOF> get_current_joint_q() const {
    std::shared_lock<std::shared_mutex> lock(fb_mutex_);
    return robot_fb_data_.current_joint_q;
  }

  Joints<DOF> get_current_joint_v() const {
    std::shared_lock<std::shared_mutex> lock(fb_mutex_);
    return robot_fb_data_.current_joint_v;
  };
  Joints<DOF> get_current_joint_t() const {
    std::shared_lock<std::shared_mutex> lock(fb_mutex_);
    return robot_fb_data_.current_joint_t;
  };
  Translation get_current_translation() const { return get_current_pose().first; };
  Rotation get_current_rotation() const { return get_current_pose().second; };
  double get_current_end() const {
    std::shared_lock<std::shared_mutex> lock(fb_mutex_);
    return robot_fb_data_.current_end;
  };

  array<uint8_t, DOF> get_current_joint_error_code() const {
    std::shared_lock<std::shared_mutex> lock(fb_mutex_);
    return robot_fb_data_.current_joint_err;
  };
  Joints<DOF> get_current_joint_temperature() const {
    std::shared_lock<std::shared_mutex> lock(fb_mutex_);
    return robot_fb_data_.current_joint_temp;
  };
  inline std::string get_sn() const { return interface_board_base_->get_arm_sn_code(); }

  bool set_target_pose(const Frame& target_pose, bool use_planning = true, double vel = DEFAULT_VEL,
                       bool blocking = false);
  bool set_target_pose(const Translation& target_translation, const Rotation& target_rotation, bool use_planning = true,
                       double vel = DEFAULT_VEL, bool blocking = false);
  bool set_target_translation(const Translation& target_translation, bool use_planning = true, double vel = DEFAULT_VEL,
                              bool blocking = false);
  bool add_target_translation(const Translation& target_d_translation, bool use_planning = true,
                              double vel = DEFAULT_VEL, bool blocking = false);
  bool add_target_relative_translation(const Translation& target_d_translation, bool use_planning = true,
                                       double vel = DEFAULT_VEL, bool blocking = false);
  bool add_target_relative_rotation(const Rotation& target_d_rotation, bool use_planning = true,
                                    double vel = DEFAULT_VEL, bool blocking = false);
  bool set_target_rotation(const Rotation& target_rotation, bool use_planning = true, double vel = DEFAULT_VEL,
                           bool blocking = false);

  bool set_target_vel(const Twist& target_vel);

  bool set_target_joint_q(const Joints<DOF>& target_joint_q, bool use_planning = true, double vel = DEFAULT_VEL,
                          bool blocking = false);
  bool add_target_joint_q(const Joints<DOF>& target_d_joint_q, bool use_planning = true, double vel = DEFAULT_VEL,
                          bool blocking = false);
  bool set_target_joint_v(const Joints<DOF>& target_joint_v);
  bool add_target_joint_v(const Joints<DOF>& target_d_joint_v);
  bool set_target_joint_mit(const Joints<DOF>& target_joint_q, const Joints<DOF>& target_joint_v,
                            const Joints<DOF>& target_joint_kp, const Joints<DOF>& target_joint_kd,
                            const Joints<DOF>& target_joint_t);
  bool set_target_joint_mit(const Joints<DOF>& target_joint_q, const Joints<DOF>& target_joint_v,
                            const Joints<DOF>& target_joint_kp, const Joints<DOF>& target_joint_kd);
  bool set_target_end(const double& end_pose, bool blocking = false);
  void set_frame(const Joints<DOF>& read_meters);

  bool valid_target_pose(const Frame& target_pose) const;
  bool valid_joint_q(const Joints<DOF>& joint_q) const;
  bool valid_joint_v(const Joints<DOF>& joint_v) const;
  bool safe_joint_q(const Joints<DOF>& joint_q) const;
  array<bool, 6> slow_joint_q(const Joints<DOF>& joint_q) const;

  void record_save(const std::string& filepath);
  void record_load(const std::string& filepath);

  inline bool record_start() { return change_mode(ArmMode::RECORDING); }

  inline bool record_stop() {
    auto current_mode = arm_mode_.load(std::memory_order_relaxed);
    if (current_mode == ArmMode::RECORDING)
      return change_mode(ArmMode::DEMONSTRATE);
    else {
      logger_->warn("current mode is {}, record_stop have no effect", ArmModeStr[current_mode]);
      return false;
    }
  }

  inline bool replay_start() {
    auto current_mode = arm_mode_.load(std::memory_order_relaxed);
    if (current_mode == ArmMode::OFFLINE || current_mode == ArmMode::REPLAY_REACHING ||
        current_mode == ArmMode::REPLAY_WAITING || current_mode == ArmMode::REPLAYING) {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      robot_cmd_data_.replay_request = true;
      return true;
    } else {
      logger_->warn("current mode is {}, replay_start have no effect", ArmModeStr[current_mode]);
      return false;
    }
  }
  /**
   * @brief test
   *
   */
  inline bool manual_mode() {
    auto current_mode = arm_mode_.load(std::memory_order_relaxed);
    if (current_mode == ArmMode::DEMONSTRATE || current_mode == ArmMode::RECORDING) {
      logger_->warn("current mode is {}, manual_mode have no effect", ArmModeStr[current_mode]);
      return false;
    } else {
      return change_mode(ArmMode::DEMONSTRATE);
    }
  }

  inline bool offline_mode() { return change_mode(ArmMode::OFFLINE); }

  inline bool online_mode() { return change_mode(ArmMode::ONLINE); }

  inline void reset_error() {
    auto mode = arm_mode_.load(std::memory_order_relaxed);
    if (mode != ArmMode::ERROR) {
      logger_->warn("current mode is {}, reset_error have no effect", ArmModeStr[mode]);
    } else {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      robot_cmd_data_.recover_request = true;
      robot_cmd_data_.cmd_state = MotorControlState::JOINT_POS;
    }
  }

  inline void set_max_current(const Joints<DOF>& max) {
    for (int i = 0; i < max.size(); i++) motor_driver_[i]->set_max_current(max[i]);
  }
};
};  // namespace arm

template class arm::Robot<6>;

#endif

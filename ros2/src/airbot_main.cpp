#include <chrono>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <airbot/command/command_base.hpp>
#include <airbot/modules/controller/fk_analytic.hpp>
#include <airbot/modules/controller/id_chain_rne.hpp>
#include <airbot/modules/controller/ik_analytic.hpp>
#include <kdl/frames.hpp>

#define QoS_CONSTANT 10

using Pose = geometry_msgs::msg::Pose;
using PosePtr = geometry_msgs::msg::Pose::UniquePtr;
using Joy = sensor_msgs::msg::Joy;
using JoyPtr = sensor_msgs::msg::Joy::UniquePtr;
using JointState = sensor_msgs::msg::JointState;
using JointStatePtr = sensor_msgs::msg::JointState::UniquePtr;
using String = std_msgs::msg::String;
using StringPtr = std_msgs::msg::String::UniquePtr;
using Bool = std_msgs::msg::Bool;
using BoolPtr = std_msgs::msg::Bool::UniquePtr;

template<typename T>
bool idle(const std::vector<T> &q) {
  for (int i = 0; i < q.size(); i++) {
    if (std::abs(q[i]) > 0.01) return false;
  }
  return true;
}

class AirbotROS2Interface : public rclcpp::Node {
 public:
  AirbotROS2Interface() : Node("airbot_arm_ros") {
    this->declare_parameter<std::string>("urdf", "");
    this->declare_parameter<std::string>("interface", "can0");
    this->declare_parameter<std::string>("end_mode", "teacher");

    urdf_path_ = this->get_parameter("urdf").as_string();
    can_if_ = this->get_parameter("interface").as_string();
    end_mode_ = this->get_parameter("end_mode").as_string();
    RCLCPP_INFO(this->get_logger(), "urdf path is %s", urdf_path_.c_str());

    air_robot_ = std::make_unique<arm::Robot>(std::make_unique<arm::AnalyticFKSolver>(urdf_path_),
                                              std::make_unique<arm::AnalyticIKSolver>(urdf_path_),
                                              std::make_unique<arm::ChainIDSolver>(urdf_path_, "down"),
                                              can_if_.c_str(),
                                              M_PI / 2,
                                              end_mode_,
                                              false,
                                              false);

    arm_pose_pub_ = this->create_publisher<Pose>("/airbot_play/arm_pose", QoS_CONSTANT);
    joint_states_pub_ = this->create_publisher<JointState>("/airbot_play/joint_states", QoS_CONSTANT);
    gripper_state_pub_ = this->create_publisher<String>("/airbot_play/gripper/state", QoS_CONSTANT);

    arm_pose_cmd_sub_ = this->create_subscription<Pose>("/airbot_play/pose_cmd", QoS_CONSTANT, [this](PosePtr pose) {
      this->air_robot_->set_target_pose({pose->position.x, pose->position.y, pose->position.z},
                                        {pose->orientation.x, pose->orientation.y, pose->orientation.z,
                                         pose->orientation.w});
    });
    joint_cmd_sub_ =
        this->create_subscription<JointState>("/airbot_play/joint_cmd", QoS_CONSTANT, [this](JointStatePtr joints) {
                                                this->air_robot_->set_target_joint_q({joints->position[0], joints->position[1], joints->position[2],
                                                                                      joints->position[3],
                                                                                      joints->position[4], joints->position[5]});
                                              }
        );
    endeffector_cmd_sub_ =
        this->create_subscription<Bool>("/airbot_play/gripper/state_cmd", QoS_CONSTANT, [this](BoolPtr end) {
                                          this->air_robot_->set_target_end(std::clamp(end->data ? 0.99 : 0.01, 0.0, 1.0));
                                        }
        );


    bool recording = false;
    joy_cmd_sub_ =
        this->create_subscription<Joy>("/joy_latched", QoS_CONSTANT, [this](const JoyPtr &joy) {
                                         bool flag = false;
                                         float scale = 0.005, angle_scale = 0.2;
                                         if (joy->buttons[7] == 0) {
                                           if (std::abs(joy->axes[1]) > 1e-3 || std::abs(joy->axes[0]) > 1e-3 || std::abs(joy->axes[3]) > 1e-3) {
                                             if (joy->buttons[6] == 1) {  // LT
                                               this->air_robot_->add_target_relative_translation(
                                                   {-scale * joy->axes[3], scale * joy->axes[0], scale * joy->axes[1]});
                                             } else {
                                               this->air_robot_->add_target_translation({scale * joy->axes[1], scale * joy->axes[0],
                                                                                         scale * joy->axes[3]});
                                             }
                                           }
                                         } else {
                                           if (std::abs(joy->axes[2]) > 1e-3 || std::abs(joy->axes[3]) > 1e-3 || joy->axes[4] != 0) {
                                             double x = 0, y = 0, z = 0, w = 0;
                                             if (joy->axes[4] == 0) {
                                               KDL::Rotation::Rot(KDL::Vector(joy->axes[2], joy->axes[3], 0),
                                                                  angle_scale * (joy->axes[3] * joy->axes[3] + joy->axes[2] * joy->axes[2]))
                                                   .GetQuaternion(x, y, z, w);
                                             } else {
                                               KDL::Rotation::Rot(KDL::Vector(0, 0, 1), -angle_scale * joy->axes[4]).GetQuaternion(x, y, z, w);
                                             }
                                             this->air_robot_->add_target_relative_rotation({x, y, z, w});
                                           }
                                         }
                                       }
        );
    joy_diff_cmd_sub_ = this->create_subscription<Joy>("/joy_trigger",
                                                       QoS_CONSTANT,
                                                       [this, &recording](const JoyPtr &joy) {
                                                         if (joy->buttons[3] == 1)
                                                           this->air_robot_->gravity_compensation();  // Y
                                                         if (joy->buttons[0] == 1) {
                                                           if (this->air_robot_->get_current_end() < 0.5)
                                                             this->air_robot_->set_target_end(0.99);
                                                           else
                                                             this->air_robot_->set_target_end(0.01);
                                                         }
                                                         if (joy->buttons[1] == 1) {                              // A
                                                           if (!recording) {
                                                             this->air_robot_->record_start("q");
                                                             RCLCPP_INFO(this->get_logger(), "record started");
                                                             recording = true;
                                                           } else {
                                                             this->air_robot_->record_stop();
                                                             RCLCPP_INFO(this->get_logger(), "record stopped");
                                                             recording = false;
                                                           }
                                                         }
                                                         if (joy->buttons[2] == 1)
                                                           this->air_robot_->record_replay();  // B
                                                       }
    );

    auto timer_callback =
        [this]() -> void {
          auto airbot_pose_msg = Pose();
          auto end_pose = this->air_robot_->get_current_pose();
          airbot_pose_msg.position.x = end_pose[0][0];
          airbot_pose_msg.position.y = end_pose[0][1];
          airbot_pose_msg.position.z = end_pose[0][2];
          airbot_pose_msg.orientation.x = end_pose[1][0];
          airbot_pose_msg.orientation.y = end_pose[1][1];
          airbot_pose_msg.orientation.z = end_pose[1][2];
          airbot_pose_msg.orientation.w = end_pose[1][3];
          arm_pose_pub_->publish(airbot_pose_msg);

          auto joint_states_msg = JointState();
          auto joint_pos = this->air_robot_->get_current_joint_q();
          auto joint_spd = this->air_robot_->get_current_joint_v();
          auto joint_eff = this->air_robot_->get_current_joint_t();
          joint_states_msg.header.stamp = this->now();;
          joint_states_msg.header.frame_id = "airbot_arm";
          joint_states_msg.position = joint_pos;
          joint_states_msg.velocity = joint_spd;
          joint_states_msg.effort = joint_eff;

          for (int motor_id = 1; motor_id <= 6; motor_id++)
            joint_states_msg.name.push_back("joint" + std::to_string(motor_id));
          joint_states_pub_->publish(joint_states_msg);

          auto end_state_msg = String();
          std::string end_status;
          auto end_pos = this->air_robot_->get_current_end();
          if (end_pos > 0.8)
            end_status = "open";
          else if (end_pos < 0.2)
            end_status = "close";
          else
            end_status = "moving";
          end_state_msg.data = end_status;

          gripper_state_pub_->publish(end_state_msg);
        };
    timer_ = this->create_wall_timer(std::chrono::milliseconds(5), timer_callback);
  }

 private:
  std::string urdf_path_;
  std::string can_if_;
  std::string end_mode_;
  std::unique_ptr<arm::Robot> air_robot_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Pose>::SharedPtr arm_pose_pub_;
  rclcpp::Publisher<JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<String>::SharedPtr gripper_state_pub_;

  rclcpp::Subscription<Pose>::SharedPtr arm_pose_cmd_sub_;
  rclcpp::Subscription<JointState>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<Bool>::SharedPtr endeffector_cmd_sub_;
  rclcpp::Subscription<Joy>::SharedPtr joy_cmd_sub_;
  rclcpp::Subscription<Joy>::SharedPtr joy_diff_cmd_sub_;

  std::mutex mutex_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AirbotROS2Interface>());
  rclcpp::shutdown();
  return 0;

}
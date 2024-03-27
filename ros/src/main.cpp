#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <airbot/airbot.hpp>
#include <kdl/frames.hpp>

using Pose = geometry_msgs::Pose;
using PosePtr = geometry_msgs::Pose::ConstPtr;
using Joy = sensor_msgs::Joy;
using JoyPtr = sensor_msgs::Joy::ConstPtr;
using JointState = sensor_msgs::JointState;
using JointStatePtr = sensor_msgs::JointState::ConstPtr;

std::string get_time() {
  auto now = std::chrono::system_clock::now();
  auto now_c = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");
  return ss.str();
}
template <typename T>
bool idle(const std::vector<T>& q) {
  for (int i = 0; i < q.size(); i++) {
    if (std::abs(q[i]) > 0.01) return false;
  }
  return true;
}

int main(int argc, char** argv) {
  Pose airbot_pose_msg;
  JointState joint_states_msg;
  Joy joy_msgs;
  std_msgs::String end_state_msg;
  /**
   * Initialize ROS node
   */
  ros::init(argc, argv, "airbot_arm_ros");
  ros::NodeHandle node;
  ros::Rate loop_rate(200);

  /**
   * Initialize arm controller
   */
  std::string urdf_path;
  std::string can_if;
  std::string end_mode;
  node.param("/airbot_arm_ros/urdf", urdf_path, std::string(""));
  node.param("/airbot_arm_ros/interface", can_if, std::string("can0"));
  node.param("/airbot_arm_ros/end_mode", end_mode, std::string("teacher"));
  ROS_WARN("urdf: %s, interface: %s, end_mode: %s", urdf_path.c_str(), can_if.c_str(), end_mode.c_str());
  arm::Robot robot(std::make_unique<arm::AnalyticFKSolver>(urdf_path),
                   std::make_unique<arm::AnalyticIKSolver>(urdf_path),
                   std::make_unique<arm::ChainIDSolver>(urdf_path, "down"), can_if.c_str(), M_PI / 2, end_mode, false, false);

  /**
   * Initialize ROS publisher and subscriber
   */
  auto arm_pose_publisher = node.advertise<Pose>("/airbot_play/arm_pose", 10);
  auto joint_states_publisher = node.advertise<JointState>("/airbot_play/joint_states", 10);
  auto end_state_publisher = node.advertise<std_msgs::String>("/airbot_play/gripper/state", 10);

  auto arm_pose_subscriber = node.subscribe<Pose>(
      "/airbot_play/pose_cmd", 10, {[&robot](const PosePtr& pose) {
        robot.set_target_pose({pose->position.x, pose->position.y, pose->position.z},
                              {pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w});
      }});
  auto joint_states_subscriber = node.subscribe<JointState>(
      "/airbot_play/joint_cmd", 10, {[&robot](const JointStatePtr& joints) {
        robot.set_target_joint_q({joints->position[0], joints->position[1], joints->position[2], joints->position[3],
                                  joints->position[4], joints->position[5]});
      }});
  auto end_subscriber = node.subscribe<std_msgs::Bool>(
      "/airbot_play/gripper/state_cmd", 10, {[&robot](const std_msgs::Bool::ConstPtr& end) {
        robot.set_target_end(std::clamp(end->data ? 0.99 : 0.01, 0.0, 1.0));
      }});
  auto scale = 0.01d;
  auto angle_scale = 0.2d;
  auto recording = false;
  auto joy_msgs_subscriber = node.subscribe<Joy>(
      "/joy_latched", 10, {[&robot, &scale, &angle_scale](const JoyPtr& joy) {
        bool flag = false;

        if (joy->buttons[7] == 0) {
          if (std::abs(joy->axes[1]) > 1e-3 || std::abs(joy->axes[0]) > 1e-3 || std::abs(joy->axes[3]) > 1e-3) {
            if (joy->buttons[6] == 1) {  // LT
              robot.add_target_relative_translation(
                  {-scale * joy->axes[3], scale * joy->axes[0], scale * joy->axes[1]});
            } else {
              robot.add_target_translation({scale * joy->axes[1], scale * joy->axes[0], scale * joy->axes[3]});
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
            robot.add_target_relative_rotation({x, y, z, w});
          }
        }
      }});
  auto joy_msgs_diff_subsciber =
      node.subscribe<Joy>("/joy_trigger", 10, [&robot, &scale, &angle_scale, &recording](const JoyPtr& joy) {
        if (joy->buttons[3] == 1) robot.gravity_compensation();  // Y
        if (joy->buttons[0] == 1) {
          if (robot.get_current_end() < 0.5)
            robot.set_target_end(0.99);
          else
            robot.set_target_end(0.01);
        }
        if (joy->buttons[1] == 1) {                              // A
          if (!recording) {
            robot.record_start("q");
            ROS_INFO("record started");
            recording = true;
          } else {
            robot.record_stop();
            ROS_INFO("record stopped");
            recording = false;
          }
        }
        if (joy->buttons[2] == 1) robot.record_replay();  // B
      });

  ROS_WARN("arm control begin");

  /**
   * Main loop
   */
  while (ros::ok()) {
    auto end_pose = robot.get_current_pose();
    airbot_pose_msg.position.x = end_pose[0][0];
    airbot_pose_msg.position.y = end_pose[0][1];
    airbot_pose_msg.position.z = end_pose[0][2];
    airbot_pose_msg.orientation.x = end_pose[1][0];
    airbot_pose_msg.orientation.y = end_pose[1][1];
    airbot_pose_msg.orientation.z = end_pose[1][2];
    airbot_pose_msg.orientation.w = end_pose[1][3];
    arm_pose_publisher.publish(airbot_pose_msg);

    auto joint_pos = robot.get_current_joint_q();
    auto joint_spd = robot.get_current_joint_v();
    auto joint_eff = robot.get_current_joint_t();
    joint_states_msg.header.stamp = ros::Time::now();
    joint_states_msg.header.frame_id = "airbot_arm";
    joint_states_msg.position = joint_pos;
    joint_states_msg.velocity = joint_spd;
    joint_states_msg.effort = joint_eff;

    for (int motor_id = 1; motor_id <= 6; motor_id++)
      joint_states_msg.name.push_back("joint" + std::to_string(motor_id));
    joint_states_publisher.publish(joint_states_msg);

    std::string end_status;
    auto end_pos = robot.get_current_end();
    if (end_pos > 0.8)
      end_status = "open";
    else if (end_pos < 0.2)
      end_status = "close";
    else
      end_status = "moving";
    end_state_msg.data = end_status;

    end_state_publisher.publish(end_state_msg);

    // Clear message
    airbot_pose_msg = Pose();
    joint_states_msg = JointState();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

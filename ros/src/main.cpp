#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <airbot/airbot.hpp>

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
  std_msgs::Float64 end_state_msg;
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
  arm::Robot<6> robot(urdf_path, can_if, "down", M_PI / 2, end_mode);

  /**
   * Initialize ROS publisher and subscriber
   */
  auto arm_pose_publisher = node.advertise<Pose>("/airbot_play/end_pose", 10);
  auto joint_states_publisher = node.advertise<JointState>("/airbot_play/joint_states", 10);
  auto end_state_publisher = node.advertise<std_msgs::Float64>("/airbot_play/gripper/position", 10);

  auto subscriber_target_pose = node.subscribe<Pose>(
      "/airbot_play/set_target_pose", 10, {[&robot](const PosePtr& pose) {
        robot.set_target_pose({pose->position.x, pose->position.y, pose->position.z},
                              {pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w});
      }});
  auto subscriber_target_joint_pos = node.subscribe<JointState>(
      "/airbot_play/set_target_joint_q", 10, {[&robot](const JointStatePtr& joints) {
        robot.set_target_joint_q({joints->position[0], joints->position[1], joints->position[2], joints->position[3],
                                  joints->position[4], joints->position[5]});
      }});
  auto subscriber_target_joint_vel = node.subscribe<JointState>(
      "/airbot_play/set_target_joint_v", 10, {[&robot](const JointStatePtr& joints) {
        robot.set_target_joint_v({joints->velocity[0], joints->velocity[1], joints->velocity[2], joints->velocity[3],
                                  joints->velocity[4], joints->velocity[5]});
      }});
  auto subscriber_target_set_position = node.subscribe<std_msgs::Float64>(
      "/airbot_play/gripper/set_position", 10, {[&robot](const std_msgs::Float64::ConstPtr& end) {
        robot.set_target_end(std::clamp(end->data ? 0.99 : 0.01, 0.0, 1.0));
      }});

  auto scale = 0.001d;
  auto angle_scale = 0.2d;
  auto recording = false;
  auto compensating = false;

  auto joy_msgs_subscriber = node.subscribe<Joy>(
      "/joy_latched", 10, {[&robot, &scale, &angle_scale](const JoyPtr& joy) {
        bool flag = false;

        if (joy->buttons[7] == 0) {
          if (std::abs(joy->axes[1]) > 1e-3 || std::abs(joy->axes[0]) > 1e-3 || std::abs(joy->axes[3]) > 1e-3) {
            if (joy->buttons[6] == 1) {  // LT
              robot.add_target_relative_translation({-scale * joy->axes[3], scale * joy->axes[0], scale * joy->axes[1]},
                                                    false);
            } else {
              robot.add_target_translation({scale * joy->axes[1], scale * joy->axes[0], scale * joy->axes[3]}, false);
            }
          }
        }
      }});
  auto joy_msgs_diff_subsciber = node.subscribe<Joy>(
      "/joy_trigger", 10, [&robot, &scale, &angle_scale, &recording, &compensating](const JoyPtr& joy) {
        if (joy->buttons[3] == 1) {
          if (!compensating)
            robot.manual_mode();
          else
            robot.online_mode();
          compensating = !compensating;
        }  // Y
        if (joy->buttons[0] == 1) {
          if (robot.get_current_end() < 0.5)
            robot.set_target_end(0.99);
          else
            robot.set_target_end(0.01);
        }
        if (joy->buttons[1] == 1) {  // A
          if (!recording) {
            robot.record_start();
            ROS_INFO("record started");
            recording = true;
          } else {
            robot.record_stop();
            ROS_INFO("record stopped");
            recording = false;
          }
        }
        if (joy->buttons[2] == 1) robot.replay_start();  // B
      });

  ROS_WARN("arm control begin");

  /**
   * Main loop
   */
  while (ros::ok()) {
    auto end_pose = robot.get_current_pose();
    airbot_pose_msg.position.x = end_pose.first[0];
    airbot_pose_msg.position.y = end_pose.first[1];
    airbot_pose_msg.position.z = end_pose.first[2];
    airbot_pose_msg.orientation.x = end_pose.second[0];
    airbot_pose_msg.orientation.y = end_pose.second[1];
    airbot_pose_msg.orientation.z = end_pose.second[2];
    airbot_pose_msg.orientation.w = end_pose.second[3];
    arm_pose_publisher.publish(airbot_pose_msg);

    auto joint_pos = robot.get_current_joint_q();
    auto joint_spd = robot.get_current_joint_v();
    auto joint_eff = robot.get_current_joint_t();
    joint_states_msg.header.stamp = ros::Time::now();
    joint_states_msg.header.frame_id = "airbot_arm";
    joint_states_msg.position = {joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5]};
    joint_states_msg.velocity = {joint_spd[0], joint_spd[1], joint_spd[2], joint_spd[3], joint_spd[4], joint_spd[5]};
    joint_states_msg.effort = {joint_eff[0], joint_eff[1], joint_eff[2], joint_eff[3], joint_eff[4], joint_eff[5]};

    for (int motor_id = 1; motor_id <= 6; motor_id++)
      joint_states_msg.name.push_back("joint" + std::to_string(motor_id));
    joint_states_publisher.publish(joint_states_msg);

    end_state_msg.data = robot.get_current_end();
    end_state_publisher.publish(end_state_msg);

    // Clear message
    airbot_pose_msg = Pose();
    joint_states_msg = JointState();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

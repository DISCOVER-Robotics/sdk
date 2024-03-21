#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <mutex>
#include <vector>

using Joy = sensor_msgs::Joy;
using JoyPtr = sensor_msgs::Joy::ConstPtr;

/**
 * FIXME This class is required due to the velocity control of arm end is not implemented
 * The class is both a latch to save the status of joystick and also a proxy publisher
 */
class JoyLatch {
  Joy joy_msgs_;
  Joy joy_msgs_diff_;
  std::mutex mutex_;
  ros::Subscriber sub_;
  ros::Publisher pub_latch_;
  ros::Publisher pub_trigger_;

  void onCall(const JoyPtr& joy_ptr) {
    // std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < joy_msgs_.buttons.size(); i++)
      joy_msgs_diff_.buttons[i] = joy_ptr->buttons[i] - joy_msgs_.buttons[i];
    pub_trigger_.publish(joy_msgs_diff_);
    joy_msgs_ = *joy_ptr;
  }

 public:
  JoyLatch(ros::NodeHandle& nh) {
    joy_msgs_.axes = {0, 0, 0, 0, 0, 0};
    joy_msgs_.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    joy_msgs_diff_.axes = {0, 0, 0, 0, 0, 0};
    joy_msgs_diff_.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    sub_ = nh.subscribe<Joy>("/joy", 10, &JoyLatch::onCall, this);
    pub_latch_ = nh.advertise<Joy>("/joy_latched", 10);
    pub_trigger_ = nh.advertise<Joy>("/joy_trigger", 10);
  }

  ~JoyLatch() = default;

  void run() {
    ros::Rate loop(200);
    while (ros::ok()) {
      // std::lock_guard<std::mutex> lock(mutex_);
      pub_latch_.publish(joy_msgs_);
      ros::spinOnce();
      loop.sleep();
    }
  }
};

int main(int argc, char** argv) {
  Joy joy_msgs;
  ROS_WARN("node inited");
  /**
   * Initialize ROS node
   */
  ros::init(argc, argv, "joy_proxy");
  ros::NodeHandle node;
  ros::Rate loop_rate(200);
  ROS_WARN("node inited");

  auto jl = JoyLatch(node);
  jl.run();
  return 0;
}

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;

class JoyLatch : public rclcpp::Node {
 public:
  JoyLatch() : Node("joy_proxy") {
    joy_msgs_.axes = {0, 0, 0, 0, 0, 0};
    joy_msgs_.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    joy_msgs_diff_.axes = {0, 0, 0, 0, 0, 0};
    joy_msgs_diff_.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    latch_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy_latched", 10);
    trigger_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy_trigger", 10);
    auto topic_callback =
        [this](sensor_msgs::msg::Joy::UniquePtr joy_ptr) -> void {
          for (int i = 0; i < joy_msgs_.buttons.size(); i++)
            joy_msgs_diff_.buttons[i] = joy_ptr->buttons[i] - joy_msgs_.buttons[i];
          trigger_publisher_->publish(joy_msgs_diff_);
          joy_msgs_ = *joy_ptr;
        };
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, topic_callback);

    auto timer_callback =
        [this]() -> void {
          latch_publisher_->publish(joy_msgs_);
        };
    timer_ = this->create_wall_timer(5ms, timer_callback);
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr latch_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr trigger_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  sensor_msgs::msg::Joy joy_msgs_;
  sensor_msgs::msg::Joy joy_msgs_diff_;
  std::mutex mutex_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyLatch>());
  rclcpp::shutdown();
  return 0;
}

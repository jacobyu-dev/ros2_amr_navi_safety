#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "arm_navi_safety_interfaces/msg/safety_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace safety_supervisor
{

class SafetyVelocityGate : public rclcpp::Node
{
public:
  SafetyVelocityGate()
  : Node("safety_velocity_gate")
  {
    const auto input_topic = declare_parameter<std::string>("input_cmd_vel_topic", "/navigation/cmd_vel");
    const auto output_topic = declare_parameter<std::string>("output_cmd_vel_topic", "/cmd_vel");
    const auto safety_topic = declare_parameter<std::string>("safety_topic", "/safety/status");
    const auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);
    const auto command_timeout_ms = declare_parameter<int>("command_timeout_ms", 250);
    const auto safety_timeout_ms = declare_parameter<int>("safety_timeout_ms", 500);
    if (input_topic.empty() || output_topic.empty() || safety_topic.empty() ||
      input_topic.front() != '/' || output_topic.front() != '/' || safety_topic.front() != '/')
    {
      throw std::invalid_argument("velocity gate topics must be absolute ROS names");
    }
    if (input_topic == output_topic) {
      throw std::invalid_argument("input and output velocity topics must differ");
    }
    if (publish_rate_hz <= 0.0 || command_timeout_ms <= 0 || safety_timeout_ms <= 0) {
      throw std::invalid_argument("velocity gate timing parameters must be positive");
    }
    command_timeout_ = std::chrono::milliseconds(command_timeout_ms);
    safety_timeout_ = std::chrono::milliseconds(safety_timeout_ms);

    output_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, rclcpp::QoS(10).reliable());
    command_input_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic, rclcpp::QoS(10).reliable(),
      std::bind(&SafetyVelocityGate::command_callback, this, std::placeholders::_1));
    safety_input_ = create_subscription<arm_navi_safety_interfaces::msg::SafetyStatus>(
      safety_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&SafetyVelocityGate::safety_callback, this, std::placeholders::_1));
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SafetyVelocityGate::publish_gated_command, this));
    RCLCPP_INFO(
      get_logger(), "Fail-safe velocity gate: %s -> %s, safety=%s", input_topic.c_str(),
      output_topic.c_str(), safety_topic.c_str());
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_command_ = *message;
    command_received_ = true;
    last_command_ = std::chrono::steady_clock::now();
  }

  void safety_callback(const arm_navi_safety_interfaces::msg::SafetyStatus::SharedPtr message)
  {
    using Status = arm_navi_safety_interfaces::msg::SafetyStatus;
    const bool permitted = message->data_valid &&
      (message->level == Status::SAFE || message->level == Status::WARNING);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      safety_permitted_ = permitted;
      safety_received_ = true;
      last_safety_ = std::chrono::steady_clock::now();
    }
    if (!permitted) {
      // An emergency transition must not wait for the periodic republisher.
      output_->publish(geometry_msgs::msg::Twist{});
    }
  }

  void publish_gated_command()
  {
    geometry_msgs::msg::Twist output;
    bool permitted = false;
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      permitted = safety_received_ && safety_permitted_ && command_received_ &&
        now - last_safety_ <= safety_timeout_ && now - last_command_ <= command_timeout_;
      if (permitted) {
        output = latest_command_;
      }
    }
    output_->publish(output);
    if (permitted != last_logged_permitted_) {
      RCLCPP_INFO(
        get_logger(), "Velocity gate %s", permitted ? "OPEN" : "CLOSED (publishing zero)");
      last_logged_permitted_ = permitted;
    }
  }

  std::mutex mutex_;
  geometry_msgs::msg::Twist latest_command_;
  bool command_received_{false};
  bool safety_received_{false};
  bool safety_permitted_{false};
  bool last_logged_permitted_{false};
  SteadyTime last_command_{};
  SteadyTime last_safety_{};
  std::chrono::milliseconds command_timeout_{250};
  std::chrono::milliseconds safety_timeout_{500};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_input_;
  rclcpp::Subscription<arm_navi_safety_interfaces::msg::SafetyStatus>::SharedPtr safety_input_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace safety_supervisor

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<safety_supervisor::SafetyVelocityGate>());
  rclcpp::shutdown();
  return 0;
}

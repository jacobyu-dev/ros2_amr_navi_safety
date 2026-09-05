#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace system_integration_test
{
namespace
{

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

double wrap_angle(double angle)
{
  constexpr double pi = 3.14159265358979323846;
  while (angle > pi) {angle -= 2.0 * pi;}
  while (angle < -pi) {angle += 2.0 * pi;}
  return angle;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

/**
 * Deterministic Gazebo-driving test double for Nav2's NavigateToPose action.
 * It is used only by the self-contained integration test when Nav2 is unavailable.
 */
class TestNavigateToPoseServer : public rclcpp::Node
{
public:
  TestNavigateToPoseServer()
  : Node("integration_test_navigate_to_pose_server")
  {
    const auto action_name = declare_parameter<std::string>("navigate_action_name", "/navigate_to_pose");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    const auto cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "/navigation/cmd_vel");
    linear_speed_ = declare_parameter<double>("linear_speed", 0.35);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.8);
    goal_tolerance_ = declare_parameter<double>("goal_position_tolerance", 0.25);
    if (action_name.empty() || odom_topic.empty() || cmd_topic.empty() ||
      action_name.front() != '/' || odom_topic.front() != '/' || cmd_topic.front() != '/' ||
      linear_speed_ <= 0.0 || angular_speed_ <= 0.0 || goal_tolerance_ <= 0.0)
    {
      throw std::invalid_argument("invalid deterministic navigator parameters");
    }
    command_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, rclcpp::QoS(10).reliable());
    odometry_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_ = *message;
        have_odom_ = true;
      });
    server_ = rclcpp_action::create_server<Action>(
      this, action_name,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<GoalHandle> handle) {
        std::thread(&TestNavigateToPoseServer::execute, this, handle).detach();
      });
    RCLCPP_WARN(
      get_logger(), "Using deterministic test NavigateToPose backend (not Nav2 production planning)");
  }

private:
  using Action = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  void stop() {command_->publish(geometry_msgs::msg::Twist{});}

  void execute(const std::shared_ptr<GoalHandle> handle)
  {
    rclcpp::WallRate rate(20.0);
    while (rclcpp::ok()) {
      if (handle->is_canceling()) {
        stop();
        handle->canceled(std::make_shared<Action::Result>());
        return;
      }
      nav_msgs::msg::Odometry odom;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!have_odom_) {
          rate.sleep();
          continue;
        }
        odom = odom_;
      }
      const auto & target = handle->get_goal()->pose.pose.position;
      const auto & current = odom.pose.pose.position;
      const double dx = target.x - current.x;
      const double dy = target.y - current.y;
      const double distance = std::hypot(dx, dy);
      auto feedback = std::make_shared<Action::Feedback>();
      feedback->current_pose.header = odom.header;
      feedback->current_pose.pose = odom.pose.pose;
      feedback->distance_remaining = static_cast<float>(distance);
      handle->publish_feedback(feedback);
      if (distance <= goal_tolerance_) {
        stop();
        handle->succeed(std::make_shared<Action::Result>());
        return;
      }

      const double desired_yaw = std::atan2(dy, dx);
      const double error = wrap_angle(desired_yaw - yaw_from_quaternion(odom.pose.pose.orientation));
      geometry_msgs::msg::Twist command;
      command.angular.z = clamp(2.0 * error, -angular_speed_, angular_speed_);
      if (std::abs(error) < 0.35) {
        command.linear.x = std::min(linear_speed_, std::max(0.08, distance * 0.6));
      }
      command_->publish(command);
      rate.sleep();
    }
  }

  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry odom_;
  bool have_odom_{false};
  double linear_speed_{0.35};
  double angular_speed_{0.8};
  double goal_tolerance_{0.25};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_;
  rclcpp_action::Server<Action>::SharedPtr server_;
};

}  // namespace system_integration_test

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<system_integration_test::TestNavigateToPoseServer>());
  rclcpp::shutdown();
  return 0;
}

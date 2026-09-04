#ifndef SYSTEM_INTEGRATION_TEST__SCENARIO_RUNNER_HPP_
#define SYSTEM_INTEGRATION_TEST__SCENARIO_RUNNER_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "arm_navi_safety_interfaces/msg/mission_status.hpp"
#include "arm_navi_safety_interfaces/msg/safety_status.hpp"
#include "arm_navi_safety_interfaces/srv/set_fault_mode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "system_integration_test/scenario_state_machine.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace system_integration_test
{

class ScenarioRunner : public rclcpp::Node
{
public:
  explicit ScenarioRunner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using SafetyStatus = arm_navi_safety_interfaces::msg::SafetyStatus;
  using MissionStatus = arm_navi_safety_interfaces::msg::MissionStatus;
  using SetFaultMode = arm_navi_safety_interfaces::srv::SetFaultMode;
  using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;
  using Trigger = std_srvs::srv::Trigger;
  using SteadyTime = std::chrono::steady_clock::time_point;

  void evaluate();
  void safety_callback(const SafetyStatus::SharedPtr message);
  void watchdog_callback(const SafetyStatus::SharedPtr message);
  void mission_callback(const MissionStatus::SharedPtr message);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message);
  void navigation_command_callback(const geometry_msgs::msg::Twist::SharedPtr message);
  void final_command_callback(const geometry_msgs::msg::Twist::SharedPtr message);
  void obstacle_callback(const std_msgs::msg::Bool::SharedPtr message);

  bool transition(ScenarioEvent event);
  void publish_state();
  void pass(const std::string & assertion);
  void fail(const std::string & reason);
  void finish();
  void start_mission();
  void resume_mission();
  void set_obstacle_pose(bool inject);
  void set_sensor_fault(bool enabled);
  void print_summary() const;
  [[nodiscard]] bool system_ready() const;
  [[nodiscard]] bool robot_moving() const;
  [[nodiscard]] bool robot_stationary() const;
  [[nodiscard]] static std::int64_t steady_now_ns();
  [[nodiscard]] static double velocity_magnitude(const geometry_msgs::msg::Twist & velocity);
  [[nodiscard]] std::string latency_ms(const std::string & start, const std::string & end) const;

  ScenarioStateMachine state_machine_;
  double step_timeout_sec_{30.0};
  double stop_velocity_threshold_{0.02};
  double movement_velocity_threshold_{0.05};
  double obstacle_distance_ahead_{0.95};
  int recovery_valid_samples_{5};
  int stationary_samples_required_{5};
  std::string obstacle_entity_name_{"integration_test_obstacle"};
  bool obstacle_injection_enabled_{true};
  bool sensor_failure_enabled_{true};

  bool have_scan_{false};
  bool have_odom_{false};
  bool have_safety_{false};
  bool have_watchdog_{false};
  bool mission_request_pending_{false};
  bool mission_start_accepted_{false};
  bool resume_request_pending_{false};
  bool resume_accepted_{false};
  bool obstacle_request_pending_{false};
  bool fault_request_pending_{false};
  bool obstacle_injected_{false};
  bool obstacle_removed_{false};
  bool sensor_fault_injected_{false};
  bool sensor_restored_{false};
  bool obstacle_detected_{false};
  bool obstacle_was_detected_{false};
  bool safety_stop_observed_{false};
  bool watchdog_fault_observed_{false};
  bool mission_paused_observed_{false};
  int safety_safe_samples_{0};
  int watchdog_healthy_samples_{0};
  int obstacle_clear_samples_{0};
  int stationary_samples_{0};
  SafetyStatus safety_;
  SafetyStatus watchdog_;
  MissionStatus mission_;
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::Twist navigation_command_;
  geometry_msgs::msg::Twist final_command_;
  SteadyTime state_entered_at_;
  bool terminal_reported_{false};
  std::vector<std::string> passed_assertions_;
  std::string failure_reason_;
  std::map<std::string, std::int64_t> timestamps_;

  rclcpp::Subscription<SafetyStatus>::SharedPtr safety_subscription_;
  rclcpp::Subscription<SafetyStatus>::SharedPtr watchdog_subscription_;
  rclcpp::Subscription<MissionStatus>::SharedPtr mission_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navigation_command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr final_command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_;
  rclcpp::Client<Trigger>::SharedPtr start_client_;
  rclcpp::Client<Trigger>::SharedPtr resume_client_;
  rclcpp::Client<SetFaultMode>::SharedPtr fault_client_;
  rclcpp::Client<SetEntityPose>::SharedPtr pose_client_;
  rclcpp::TimerBase::SharedPtr evaluation_timer_;
};

}  // namespace system_integration_test

#endif  // SYSTEM_INTEGRATION_TEST__SCENARIO_RUNNER_HPP_

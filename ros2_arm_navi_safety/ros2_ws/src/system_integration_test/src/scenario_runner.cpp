#include "system_integration_test/scenario_runner.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "ros_gz_interfaces/msg/entity.hpp"

namespace system_integration_test
{

ScenarioRunner::ScenarioRunner(const rclcpp::NodeOptions & options)
: Node("integration_scenario_runner", options), state_entered_at_(std::chrono::steady_clock::now())
{
  step_timeout_sec_ = declare_parameter<double>("scenario_step_timeout_sec", 30.0);
  stop_velocity_threshold_ = declare_parameter<double>("stop_velocity_threshold", 0.02);
  movement_velocity_threshold_ = declare_parameter<double>("movement_velocity_threshold", 0.05);
  obstacle_distance_ahead_ = declare_parameter<double>("obstacle_distance_ahead", 0.95);
  recovery_valid_samples_ = declare_parameter<int>("recovery_valid_samples", 5);
  stationary_samples_required_ = declare_parameter<int>("stationary_samples", 5);
  obstacle_entity_name_ = declare_parameter<std::string>(
    "obstacle_entity_name", "integration_test_obstacle");
  obstacle_injection_enabled_ = declare_parameter<bool>("obstacle_injection_enabled", true);
  sensor_failure_enabled_ = declare_parameter<bool>("sensor_failure_enabled", true);
  const auto mission_start_service = declare_parameter<std::string>("mission_start_service", "/mission/start");
  const auto mission_resume_service = declare_parameter<std::string>("mission_resume_service", "/mission/resume");
  const auto fault_service = declare_parameter<std::string>(
    "fault_service", "/fake_lidar/set_fault_mode");
  const auto set_pose_service = declare_parameter<std::string>(
    "set_entity_pose_service", "/world/amr_empty/set_pose");
  if (step_timeout_sec_ <= 0.0 || stop_velocity_threshold_ < 0.0 ||
    movement_velocity_threshold_ <= stop_velocity_threshold_ || obstacle_distance_ahead_ <= 0.0 ||
    recovery_valid_samples_ <= 1 || stationary_samples_required_ <= 0 || obstacle_entity_name_.empty())
  {
    throw std::invalid_argument("invalid integration scenario parameters");
  }

  const auto reliable = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
  safety_subscription_ = create_subscription<SafetyStatus>(
    "/safety/status", reliable,
    std::bind(&ScenarioRunner::safety_callback, this, std::placeholders::_1));
  watchdog_subscription_ = create_subscription<SafetyStatus>(
    "/safety/watchdog/status", reliable,
    std::bind(&ScenarioRunner::watchdog_callback, this, std::placeholders::_1));
  mission_subscription_ = create_subscription<MissionStatus>(
    "/mission/status", reliable,
    std::bind(&ScenarioRunner::mission_callback, this, std::placeholders::_1));
  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SensorDataQoS(),
    std::bind(&ScenarioRunner::odom_callback, this, std::placeholders::_1));
  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::LaserScan::SharedPtr) {
      have_scan_ = true;
    });
  navigation_command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "/navigation/cmd_vel", reliable,
    std::bind(&ScenarioRunner::navigation_command_callback, this, std::placeholders::_1));
  final_command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", reliable,
    std::bind(&ScenarioRunner::final_command_callback, this, std::placeholders::_1));
  obstacle_subscription_ = create_subscription<std_msgs::msg::Bool>(
    "/safety/lidar/obstacle_detected", rclcpp::QoS(10),
    std::bind(&ScenarioRunner::obstacle_callback, this, std::placeholders::_1));
  state_publisher_ = create_publisher<std_msgs::msg::String>(
    "/integration_test/scenario_state", rclcpp::QoS(10).reliable().transient_local());
  result_publisher_ = create_publisher<std_msgs::msg::String>(
    "/integration_test/result", rclcpp::QoS(1).reliable().transient_local());
  start_client_ = create_client<Trigger>(mission_start_service);
  resume_client_ = create_client<Trigger>(mission_resume_service);
  fault_client_ = create_client<SetFaultMode>(fault_service);
  pose_client_ = create_client<SetEntityPose>(set_pose_service);
  evaluation_timer_ = create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&ScenarioRunner::evaluate, this));
  publish_state();
  RCLCPP_INFO(get_logger(), "Event-driven safety integration scenario runner started");
}

std::int64_t ScenarioRunner::steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

double ScenarioRunner::velocity_magnitude(const geometry_msgs::msg::Twist & velocity)
{
  return std::max(std::hypot(velocity.linear.x, velocity.linear.y), std::abs(velocity.angular.z));
}

void ScenarioRunner::safety_callback(const SafetyStatus::SharedPtr message)
{
  safety_ = *message;
  have_safety_ = true;
  const bool safe = message->data_valid &&
    (message->level == SafetyStatus::SAFE || message->level == SafetyStatus::WARNING);
  safety_safe_samples_ = safe ? safety_safe_samples_ + 1 : 0;
  if (message->level == SafetyStatus::STOP || message->level == SafetyStatus::ERROR ||
    !message->data_valid)
  {
    safety_stop_observed_ = true;
    const auto key = state_machine_.state() == ScenarioState::WAITING_FOR_SENSOR_STOP ?
      "sensor_safety_decision" : "safety_decision";
    if (timestamps_.count(key) == 0U) {
      timestamps_[key] = message->decision_time_steady_ns > 0 ?
        message->decision_time_steady_ns : steady_now_ns();
    }
  }
}

void ScenarioRunner::watchdog_callback(const SafetyStatus::SharedPtr message)
{
  watchdog_ = *message;
  have_watchdog_ = true;
  const bool healthy = message->data_valid && message->level == SafetyStatus::SAFE;
  watchdog_healthy_samples_ = healthy ? watchdog_healthy_samples_ + 1 : 0;
  if (!healthy && message->reason.find("lidar=TIMEOUT") != std::string::npos) {
    watchdog_fault_observed_ = true;
    if (timestamps_.count("watchdog_timeout") == 0U) {
      timestamps_["watchdog_timeout"] = message->detection_time_steady_ns > 0 ?
        message->detection_time_steady_ns : steady_now_ns();
    }
  }
}

void ScenarioRunner::mission_callback(const MissionStatus::SharedPtr message)
{
  mission_ = *message;
  if (message->mission_state == MissionStatus::PAUSED) {
    mission_paused_observed_ = true;
  }
}

void ScenarioRunner::odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
{
  odom_ = *message;
  have_odom_ = true;
  stationary_samples_ = velocity_magnitude(message->twist.twist) <= stop_velocity_threshold_ ?
    stationary_samples_ + 1 : 0;
}

void ScenarioRunner::navigation_command_callback(const geometry_msgs::msg::Twist::SharedPtr message)
{
  navigation_command_ = *message;
}

void ScenarioRunner::final_command_callback(const geometry_msgs::msg::Twist::SharedPtr message)
{
  final_command_ = *message;
  const auto key = state_machine_.state() == ScenarioState::WAITING_FOR_SENSOR_STOP ?
    "sensor_stop_command" : "obstacle_stop_command";
  if (safety_stop_observed_ && velocity_magnitude(*message) <= stop_velocity_threshold_ &&
    timestamps_.count(key) == 0U)
  {
    timestamps_[key] = steady_now_ns();
  }
}

void ScenarioRunner::obstacle_callback(const std_msgs::msg::Bool::SharedPtr message)
{
  obstacle_detected_ = message->data;
  obstacle_was_detected_ = obstacle_was_detected_ || message->data;
  obstacle_clear_samples_ = message->data ? 0 : obstacle_clear_samples_ + 1;
  if (message->data && timestamps_.count("obstacle_detection") == 0U) {
    timestamps_["obstacle_detection"] = steady_now_ns();
  }
}

bool ScenarioRunner::system_ready() const
{
  const bool safety_ready = have_safety_ && safety_.data_valid &&
    (safety_.level == SafetyStatus::SAFE || safety_.level == SafetyStatus::WARNING);
  const bool watchdog_ready = have_watchdog_ && watchdog_.data_valid &&
    watchdog_.level == SafetyStatus::SAFE;
  return have_scan_ && have_odom_ && safety_ready && watchdog_ready &&
         start_client_->service_is_ready() && resume_client_->service_is_ready() &&
         fault_client_->service_is_ready() && pose_client_->service_is_ready();
}

bool ScenarioRunner::robot_moving() const
{
  return velocity_magnitude(navigation_command_) > movement_velocity_threshold_ &&
         velocity_magnitude(final_command_) > movement_velocity_threshold_ &&
         velocity_magnitude(odom_.twist.twist) > movement_velocity_threshold_;
}

bool ScenarioRunner::robot_stationary() const
{
  return stationary_samples_ >= stationary_samples_required_ &&
         velocity_magnitude(final_command_) <= stop_velocity_threshold_;
}

bool ScenarioRunner::transition(ScenarioEvent event)
{
  const auto before = state_machine_.state();
  if (!state_machine_.handle(event)) {
    return false;
  }
  if (state_machine_.state() != before) {
    state_entered_at_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(), "Scenario state: %s -> %s", to_string(before), to_string(state_machine_.state()));
    publish_state();
  }
  return true;
}

void ScenarioRunner::publish_state()
{
  std_msgs::msg::String message;
  message.data = to_string(state_machine_.state());
  state_publisher_->publish(message);
}

void ScenarioRunner::pass(const std::string & assertion)
{
  if (std::find(passed_assertions_.begin(), passed_assertions_.end(), assertion) ==
    passed_assertions_.end())
  {
    passed_assertions_.push_back(assertion);
    RCLCPP_INFO(get_logger(), "[PASS] %s", assertion.c_str());
  }
}

void ScenarioRunner::fail(const std::string & reason)
{
  if (state_machine_.terminal()) {
    return;
  }
  failure_reason_ = reason;
  RCLCPP_ERROR(get_logger(), "[FAIL] %s", reason.c_str());
  transition(ScenarioEvent::CRITICAL_FAILURE);
  finish();
}

void ScenarioRunner::start_mission()
{
  if (mission_request_pending_ || mission_start_accepted_) {return;}
  mission_request_pending_ = true;
  start_client_->async_send_request(std::make_shared<Trigger::Request>(),
    [this](rclcpp::Client<Trigger>::SharedFuture future) {
      mission_request_pending_ = false;
      if (!future.get()->success) {
        RCLCPP_WARN(get_logger(), "Mission start not accepted yet: %s", future.get()->message.c_str());
      } else {
        mission_start_accepted_ = true;
      }
    });
}

void ScenarioRunner::resume_mission()
{
  if (resume_request_pending_ || resume_accepted_) {return;}
  resume_request_pending_ = true;
  resume_client_->async_send_request(std::make_shared<Trigger::Request>(),
    [this](rclcpp::Client<Trigger>::SharedFuture future) {
      resume_request_pending_ = false;
      if (!future.get()->success) {
        RCLCPP_WARN(get_logger(), "Mission resume not accepted yet: %s", future.get()->message.c_str());
      } else {
        resume_accepted_ = true;
      }
    });
}

void ScenarioRunner::set_obstacle_pose(bool inject)
{
  if (obstacle_request_pending_) {return;}
  obstacle_request_pending_ = true;
  auto request = std::make_shared<SetEntityPose::Request>();
  request->entity.name = obstacle_entity_name_;
  request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
  if (inject) {
    const auto & q = odom_.pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    request->pose.position.x = odom_.pose.pose.position.x + obstacle_distance_ahead_ * std::cos(yaw);
    request->pose.position.y = odom_.pose.pose.position.y + obstacle_distance_ahead_ * std::sin(yaw);
    request->pose.position.z = 0.5;
  } else {
    request->pose.position.x = 100.0;
    request->pose.position.y = 100.0;
    request->pose.position.z = 0.5;
  }
  request->pose.orientation.w = 1.0;
  pose_client_->async_send_request(request,
    [this, inject](rclcpp::Client<SetEntityPose>::SharedFuture future) {
      obstacle_request_pending_ = false;
      if (!future.get()->success) {
        RCLCPP_WARN(get_logger(), "Gazebo obstacle pose request failed; retrying until step timeout");
        return;
      }
      if (inject) {
        obstacle_was_detected_ = false;
        safety_stop_observed_ = false;
        mission_paused_observed_ = false;
        stationary_samples_ = 0;
        timestamps_.erase("safety_decision");
        timestamps_.erase("obstacle_stop_command");
        obstacle_injected_ = true;
        timestamps_["obstacle_injection"] = steady_now_ns();
        pass("Obstacle injected");
        transition(ScenarioEvent::OBSTACLE_INJECTED);
      } else {
        obstacle_removed_ = true;
        timestamps_["obstacle_removed"] = steady_now_ns();
        pass("Obstacle removed");
        static_cast<void>(state_machine_.handle(ScenarioEvent::OBSTACLE_REMOVED));
      }
    });
}

void ScenarioRunner::set_sensor_fault(bool enabled)
{
  if (fault_request_pending_) {return;}
  fault_request_pending_ = true;
  auto request = std::make_shared<SetFaultMode::Request>();
  request->mode = enabled ? SetFaultMode::Request::STOP_PUBLISH : SetFaultMode::Request::NORMAL;
  fault_client_->async_send_request(request,
    [this, enabled](rclcpp::Client<SetFaultMode>::SharedFuture future) {
      fault_request_pending_ = false;
      if (!future.get()->success) {
        RCLCPP_WARN(get_logger(), "Sensor fault request failed; retrying until step timeout");
        return;
      }
      if (enabled) {
        safety_stop_observed_ = false;
        watchdog_fault_observed_ = false;
        mission_paused_observed_ = false;
        stationary_samples_ = 0;
        timestamps_.erase("sensor_safety_decision");
        timestamps_.erase("sensor_stop_command");
        sensor_fault_injected_ = true;
        timestamps_["sensor_failure_injection"] = steady_now_ns();
        pass("Sensor fault injected");
        static_cast<void>(state_machine_.handle(ScenarioEvent::SENSOR_FAULT_INJECTED));
      } else {
        sensor_restored_ = true;
        timestamps_["sensor_restore"] = steady_now_ns();
        pass("Sensor restored");
        static_cast<void>(state_machine_.handle(ScenarioEvent::SENSOR_RESTORED));
      }
    });
}

void ScenarioRunner::evaluate()
{
  if (state_machine_.terminal()) {finish(); return;}
  if (std::chrono::duration<double>(std::chrono::steady_clock::now() - state_entered_at_).count() >
    step_timeout_sec_)
  {
    failure_reason_ = "step timeout in " + std::string(to_string(state_machine_.state()));
    RCLCPP_ERROR(get_logger(), "%s", failure_reason_.c_str());
    transition(ScenarioEvent::STEP_TIMEOUT);
    finish();
    return;
  }

  switch (state_machine_.state()) {
    case ScenarioState::INITIALIZING:
      if (system_ready()) {
        pass("ROS / Gazebo initialized");
        transition(ScenarioEvent::SYSTEM_READY);
        start_mission();
      }
      break;
    case ScenarioState::WAITING_FOR_NAVIGATION:
      start_mission();
      if (mission_.mission_state == MissionStatus::RUNNING &&
        mission_.navigation_state == MissionStatus::NAVIGATING)
      {
        pass("NavigateToPose accepted");
        transition(ScenarioEvent::MISSION_ACCEPTED);
      }
      break;
    case ScenarioState::NORMAL_NAVIGATION:
      if (robot_moving()) {
        pass("Normal navigation started");
        if (obstacle_injection_enabled_) {
          set_obstacle_pose(true);
        } else {
          fail("obstacle injection is disabled for the required full scenario");
        }
      }
      break;
    case ScenarioState::WAITING_FOR_OBSTACLE_STOP:
      if (obstacle_was_detected_) {pass("Obstacle detected");}
      if (safety_stop_observed_) {pass("Safety stop triggered");}
      if (obstacle_was_detected_ && safety_stop_observed_ && mission_paused_observed_ && robot_stationary()) {
        timestamps_["robot_stationary_obstacle"] = steady_now_ns();
        pass("Robot stopped for obstacle");
        transition(ScenarioEvent::OBSTACLE_STOP_CONFIRMED);
        safety_safe_samples_ = 0;
        watchdog_healthy_samples_ = 0;
        obstacle_clear_samples_ = 0;
        set_obstacle_pose(false);
      }
      break;
    case ScenarioState::WAITING_FOR_OBSTACLE_RECOVERY:
      if (!obstacle_removed_) {set_obstacle_pose(false); break;}
      if (!obstacle_detected_ && obstacle_clear_samples_ >= recovery_valid_samples_ &&
        safety_safe_samples_ >= recovery_valid_samples_ &&
        watchdog_healthy_samples_ >= recovery_valid_samples_)
      {
        timestamps_["obstacle_recovery"] = steady_now_ns();
        pass("Safety recovered after stable obstacle clearance");
        transition(ScenarioEvent::OBSTACLE_RECOVERED);
        resume_accepted_ = false;
        resume_mission();
      }
      break;
    case ScenarioState::WAITING_FOR_OBSTACLE_RESUME:
      resume_mission();
      if (mission_.mission_state == MissionStatus::RUNNING &&
        mission_.navigation_state == MissionStatus::NAVIGATING && robot_moving())
      {
        timestamps_["obstacle_navigation_resume"] = steady_now_ns();
        pass("Navigation resumed after obstacle");
        transition(ScenarioEvent::NAVIGATION_RESUMED);
        safety_stop_observed_ = false;
        mission_paused_observed_ = false;
        stationary_samples_ = 0;
        set_sensor_fault(true);
      }
      break;
    case ScenarioState::WAITING_FOR_SENSOR_STOP:
      if (!sensor_fault_injected_) {set_sensor_fault(true); break;}
      if (watchdog_fault_observed_) {pass("Watchdog timeout detected");}
      if (safety_stop_observed_) {pass("Safety sensor fault triggered");}
      if (watchdog_fault_observed_ && safety_stop_observed_ && mission_paused_observed_ &&
        robot_stationary())
      {
        timestamps_["robot_stationary_sensor"] = steady_now_ns();
        pass("Robot stopped for sensor fault");
        transition(ScenarioEvent::SENSOR_STOP_CONFIRMED);
        safety_safe_samples_ = 0;
        watchdog_healthy_samples_ = 0;
        set_sensor_fault(false);
      }
      break;
    case ScenarioState::WAITING_FOR_SENSOR_RECOVERY:
      if (!sensor_restored_) {set_sensor_fault(false); break;}
      if (watchdog_healthy_samples_ >= recovery_valid_samples_ &&
        safety_safe_samples_ >= recovery_valid_samples_)
      {
        timestamps_["sensor_recovery"] = steady_now_ns();
        pass("Watchdog healthy after stable samples");
        pass("Safety recovered after sensor restore");
        transition(ScenarioEvent::SENSOR_RECOVERED);
        resume_accepted_ = false;
        resume_mission();
      }
      break;
    case ScenarioState::WAITING_FOR_SENSOR_RESUME:
      resume_mission();
      if (mission_.mission_state == MissionStatus::RUNNING &&
        mission_.navigation_state == MissionStatus::NAVIGATING && robot_moving())
      {
        timestamps_["sensor_navigation_resume"] = steady_now_ns();
        pass("Navigation resumed after sensor recovery");
        transition(ScenarioEvent::NAVIGATION_RESUMED);
      }
      break;
    case ScenarioState::FINAL_NAVIGATION:
      if (mission_.mission_state == MissionStatus::COMPLETED) {
        timestamps_["goal_reached"] = steady_now_ns();
        pass("Goal reached");
        transition(ScenarioEvent::GOAL_REACHED);
        finish();
      }
      break;
    case ScenarioState::PASSED:
    case ScenarioState::FAILED:
      finish();
      break;
  }
}

std::string ScenarioRunner::latency_ms(const std::string & start, const std::string & end) const
{
  const auto first = timestamps_.find(start);
  const auto second = timestamps_.find(end);
  if (first == timestamps_.end() || second == timestamps_.end() || second->second < first->second) {
    return "N/A";
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) <<
    static_cast<double>(second->second - first->second) / 1.0e6 << " ms";
  return stream.str();
}

void ScenarioRunner::print_summary() const
{
  std::ostringstream summary;
  summary << "\n================================================\n"
          << "Safety System Integration Scenario\n"
          << "================================================\n";
  for (const auto & assertion : passed_assertions_) {
    summary << "  [PASS] " << assertion << '\n';
  }
  if (!failure_reason_.empty()) {summary << "  [FAIL] " << failure_reason_ << '\n';}
  summary << "\nLatency\n"
          << "  Obstacle detection: " << latency_ms("obstacle_injection", "obstacle_detection") << '\n'
          << "  Obstacle safety decision: " << latency_ms("obstacle_injection", "safety_decision") << '\n'
          << "  Obstacle stop command: " << latency_ms("obstacle_injection", "obstacle_stop_command") << '\n'
          << "  Obstacle stationary: " << latency_ms("obstacle_injection", "robot_stationary_obstacle") << '\n'
          << "  Obstacle recovery: " << latency_ms("obstacle_removed", "obstacle_recovery") << '\n'
          << "  Watchdog timeout: " << latency_ms("sensor_failure_injection", "watchdog_timeout") << '\n'
          << "  Sensor safety decision: " << latency_ms("sensor_failure_injection", "sensor_safety_decision") << '\n'
          << "  Sensor stop command: " << latency_ms("sensor_failure_injection", "sensor_stop_command") << '\n'
          << "  Sensor recovery: " << latency_ms("sensor_restore", "sensor_recovery") << '\n'
          << "------------------------------------------------\n"
          << "Result: " << (state_machine_.passed() ? "PASS" : "FAIL") << '\n'
          << "------------------------------------------------";
  RCLCPP_INFO(get_logger(), "%s", summary.str().c_str());
}

void ScenarioRunner::finish()
{
  if (terminal_reported_) {return;}
  terminal_reported_ = true;
  print_summary();
  std_msgs::msg::String result;
  result.data = state_machine_.passed() ? "PASS" : "FAIL: " + failure_reason_;
  result_publisher_->publish(result);
}

}  // namespace system_integration_test

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<system_integration_test::ScenarioRunner>());
  rclcpp::shutdown();
  return 0;
}

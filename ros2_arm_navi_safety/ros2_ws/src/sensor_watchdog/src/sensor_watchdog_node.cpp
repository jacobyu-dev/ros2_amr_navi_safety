#include "sensor_watchdog/watchdog_monitor.hpp"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "arm_navi_safety_interfaces/msg/safety_status.hpp"
#include "arm_navi_safety_interfaces/msg/safety_event.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace sensor_watchdog
{
namespace
{
using SafetyStatus = arm_navi_safety_interfaces::msg::SafetyStatus;
using SafetyEvent = arm_navi_safety_interfaces::msg::SafetyEvent;

struct LatencyFaultEvent
{
  std::uint64_t event_id{0U};
  std::string fault_type;
  std::int64_t fault_time_steady_ns{0};
};

std::int64_t steady_now_ns() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

rclcpp::QoS safety_status_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

std::chrono::milliseconds positive_milliseconds(
  rclcpp::Node & node, const std::string & parameter_name, std::int64_t default_value)
{
  const auto value = node.declare_parameter<std::int64_t>(parameter_name, default_value);
  if (value <= 0) {
    throw std::invalid_argument(parameter_name + " must be positive");
  }
  return std::chrono::milliseconds{value};
}

std::string status_reason(const WatchdogEvaluation & evaluation)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < evaluation.sources.size(); ++index) {
    if (index != 0U) {
      stream << "; ";
    }
    stream << evaluation.sources[index].source_name << "=" << to_string(evaluation.sources[index].state);
  }
  return stream.str();
}

}  // namespace

/**
 * @brief Detects missing LiDAR input and missing TF-monitor reports.
 *
 * Input callbacks may overlap under a MultiThreadedExecutor. WatchdogMonitor
 * owns the mutex for its compound timestamp/received/state records, while the
 * timer callback group serializes logging and output state transitions.
 */
class SensorWatchdogNode : public rclcpp::Node
{
public:
  SensorWatchdogNode()
  : Node("sensor_watchdog_node"),
    lidar_timeout_(positive_milliseconds(*this, "lidar_timeout_ms", 500)),
    tf_timeout_(positive_milliseconds(*this, "tf_timeout_ms", 500)),
    check_period_(positive_milliseconds(*this, "watchdog_check_period_ms", 100)),
    startup_grace_period_(read_startup_grace_period()),
    monitor_(
      {{"lidar", lidar_timeout_, true}, {"tf_monitor", tf_timeout_, true}},
      startup_grace_period_, this->now().nanoseconds())
  {
    const auto scan_topic = this->declare_parameter<std::string>("scan_topic", "/scan");
    const auto tf_status_topic = this->declare_parameter<std::string>(
      "tf_status_topic", "/safety/tf/status");
    const auto watchdog_status_topic = this->declare_parameter<std::string>(
      "watchdog_status_topic", "/safety/watchdog/status");
    const auto fault_event_topic = this->declare_parameter<std::string>(
      "fault_event_topic", "/safety/fault_injection/events");
    const auto configured_executor_threads = this->declare_parameter<int>("executor_threads", 4);
    validate_configuration(
      scan_topic, tf_status_topic, watchdog_status_topic, configured_executor_threads);
    validate_topic(fault_event_topic, "fault_event_topic");
    executor_threads_ = static_cast<std::size_t>(configured_executor_threads);

    input_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions input_options;
    input_options.callback_group = input_callback_group_;
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr) {record_update("lidar");}, input_options);
    tf_status_subscription_ = this->create_subscription<SafetyStatus>(
      tf_status_topic, safety_status_qos(),
      [this](const SafetyStatus::SharedPtr) {record_update("tf_monitor");}, input_options);
    fault_event_subscription_ = this->create_subscription<SafetyEvent>(
      fault_event_topic, safety_status_qos(),
      std::bind(&SensorWatchdogNode::fault_event_callback, this, std::placeholders::_1), input_options);
    status_publisher_ = this->create_publisher<SafetyStatus>(watchdog_status_topic, safety_status_qos());
    watchdog_timer_ = this->create_wall_timer(
      check_period_, std::bind(&SensorWatchdogNode::check_timeouts, this), timer_callback_group_);

    RCLCPP_INFO(
      this->get_logger(),
      "Sensor watchdog started: LiDAR timeout=%ld ms, TF monitor timeout=%ld ms, startup grace=%ld ms",
      lidar_timeout_.count(), tf_timeout_.count(), startup_grace_period_.count());
  }

  [[nodiscard]] std::size_t executor_threads() const noexcept
  {
    return executor_threads_;
  }

private:
  [[nodiscard]] std::chrono::milliseconds read_startup_grace_period()
  {
    const auto value = this->declare_parameter<std::int64_t>("startup_grace_period_ms", 2000);
    if (value < 0) {
      throw std::invalid_argument("startup_grace_period_ms must be non-negative");
    }
    return std::chrono::milliseconds{value};
  }

  static void validate_topic(const std::string & topic, const std::string & parameter_name)
  {
    if (topic.empty() || topic.front() != '/') {
      throw std::invalid_argument(parameter_name + " must be an absolute ROS topic");
    }
  }

  static void validate_configuration(
    const std::string & scan_topic, const std::string & tf_status_topic,
    const std::string & watchdog_status_topic, int configured_executor_threads)
  {
    validate_topic(scan_topic, "scan_topic");
    validate_topic(tf_status_topic, "tf_status_topic");
    validate_topic(watchdog_status_topic, "watchdog_status_topic");
    if (configured_executor_threads <= 0) {
      throw std::invalid_argument("executor_threads must be positive");
    }
  }

  void record_update(const std::string & source_name)
  {
    monitor_.record_update(source_name, this->now().nanoseconds());
    input_callback_count_.fetch_add(1U);
  }

  void fault_event_callback(const SafetyEvent::SharedPtr message)
  {
    if (message->source != "lidar" || message->event_id == 0U ||
      message->fault_time_steady_ns <= 0 || message->fault_type != "lidar_timeout")
    {
      return;
    }
    std::lock_guard<std::mutex> lock(latency_event_mutex_);
    latest_lidar_fault_ = LatencyFaultEvent{
      message->event_id, message->fault_type, message->fault_time_steady_ns};
  }

  [[nodiscard]] std::optional<LatencyFaultEvent> latest_lidar_fault() const
  {
    std::lock_guard<std::mutex> lock(latency_event_mutex_);
    return latest_lidar_fault_;
  }

  void check_timeouts()
  {
    const auto now = this->now();
    const auto evaluation = monitor_.evaluate(now.nanoseconds());
    log_transitions(evaluation, now.nanoseconds());

    SafetyStatus status;
    status.header.stamp = now;
    status.source = "sensor_watchdog";
    status.reason = status_reason(evaluation);
    status.data_valid = evaluation.overall_state == WatchdogState::HEALTHY;
    status.level = evaluation.overall_state == WatchdogState::HEALTHY ? SafetyStatus::SAFE :
      evaluation.overall_state == WatchdogState::WAITING ? SafetyStatus::UNKNOWN : SafetyStatus::ERROR;
    const auto lidar = std::find_if(
      evaluation.sources.begin(), evaluation.sources.end(),
      [](const WatchdogEntry & entry) {return entry.source_name == "lidar";});
    if (lidar != evaluation.sources.end() && lidar->state == WatchdogState::TIMEOUT) {
      const auto event = latest_lidar_fault();
      if (event.has_value()) {
        // T1 is the timer instant that first observes LiDAR timeout policy violation.
        status.latency_event_id = event->event_id;
        status.latency_fault_type = event->fault_type;
        status.fault_time_steady_ns = event->fault_time_steady_ns;
        status.detection_time_steady_ns = steady_now_ns();
      }
    }
    status_publisher_->publish(status);
    timer_callback_count_.fetch_add(1U);
  }

  void log_transitions(const WatchdogEvaluation & evaluation, std::int64_t now_ns)
  {
    for (const auto & entry : evaluation.sources) {
      const auto previous = last_logged_states_.find(entry.source_name);
      if (previous != last_logged_states_.end() && previous->second == entry.state) {
        continue;
      }
      last_logged_states_[entry.source_name] = entry.state;
      if (entry.state == WatchdogState::TIMEOUT) {
        const auto age_ms = entry.received_once ?
          (now_ns > entry.last_update_time_ns ? (now_ns - entry.last_update_time_ns) / 1000000LL : 0LL) :
          startup_grace_period_.count();
        RCLCPP_ERROR(
          this->get_logger(), "%s watchdog timeout detected: %lld ms", entry.source_name.c_str(), age_ms);
      } else if (entry.state == WatchdogState::HEALTHY) {
        RCLCPP_INFO(this->get_logger(), "%s watchdog healthy/recovered", entry.source_name.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "%s watchdog waiting for first update", entry.source_name.c_str());
      }
    }
  }

  const std::chrono::milliseconds lidar_timeout_;
  const std::chrono::milliseconds tf_timeout_;
  const std::chrono::milliseconds check_period_;
  const std::chrono::milliseconds startup_grace_period_;
  WatchdogMonitor monitor_;
  std::size_t executor_threads_{4U};
  rclcpp::CallbackGroup::SharedPtr input_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<SafetyStatus>::SharedPtr tf_status_subscription_;
  rclcpp::Subscription<SafetyEvent>::SharedPtr fault_event_subscription_;
  rclcpp::Publisher<SafetyStatus>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::unordered_map<std::string, WatchdogState> last_logged_states_;
  mutable std::mutex latency_event_mutex_;
  std::optional<LatencyFaultEvent> latest_lidar_fault_;
  std::atomic<std::uint64_t> input_callback_count_{0U};
  std::atomic<std::uint64_t> timer_callback_count_{0U};
};

}  // namespace sensor_watchdog

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<sensor_watchdog::SensorWatchdogNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), node->executor_threads());
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

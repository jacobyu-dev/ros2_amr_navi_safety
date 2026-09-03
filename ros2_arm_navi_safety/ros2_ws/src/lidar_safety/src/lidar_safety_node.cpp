#include "lidar_safety/lidar_safety_core.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "arm_navi_safety_interfaces/msg/safety_status.hpp"
#include "arm_navi_safety_interfaces/msg/safety_event.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

namespace lidar_safety
{
namespace
{
std::int64_t steady_now_ns() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct LatencyFaultEvent
{
  std::uint64_t event_id{0U};
  std::string fault_type;
  std::int64_t fault_time_steady_ns{0};
};
}  // namespace

/**
 * @brief Converts LaserScan input into obstacle and SafetyStatus outputs.
 *
 * Scan evaluation is stateless, so the subscription is reentrant. Only the
 * previous-obstacle value used to suppress repeated transition logs is shared.
 */
class LidarSafetyNode : public rclcpp::Node
{
public:
  /// Declare immutable scan configuration and create safety topic endpoints.
  LidarSafetyNode()
  : Node("lidar_safety_node")
  {
    const std::string scan_topic = this->declare_parameter<std::string>("scan_topic", "/scan");
    const std::string fault_event_topic = this->declare_parameter<std::string>(
      "fault_event_topic", "/safety/fault_injection/events");
    config_.stop_distance_m = this->declare_parameter<float>("stop_distance_m", 0.7F);
    config_.front_angle_deg = this->declare_parameter<float>("front_angle_deg", 30.0F);
    const auto configured_executor_threads = this->declare_parameter<int>("executor_threads", 4);
    validate_configuration(scan_topic);
    if (fault_event_topic.empty() || fault_event_topic.front() != '/') {
      throw std::invalid_argument("fault_event_topic must be an absolute ROS topic");
    }
    if (configured_executor_threads <= 0) {
      throw std::invalid_argument("executor_threads must be positive");
    }
    executor_threads_ = static_cast<std::size_t>(configured_executor_threads);

    obstacle_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
      "/safety/lidar/obstacle_detected", rclcpp::QoS(10));
    min_distance_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
      "/safety/lidar/min_distance", rclcpp::QoS(10));
    safety_status_publisher_ = this->create_publisher<
      arm_navi_safety_interfaces::msg::SafetyStatus>(
      "/safety/lidar/status", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    // Scan processing is stateless; only the transition-log state is shared.
    scan_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = scan_callback_group_;
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarSafetyNode::scan_callback, this, std::placeholders::_1), subscription_options);
    fault_event_subscription_ = this->create_subscription<arm_navi_safety_interfaces::msg::SafetyEvent>(
      fault_event_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&LidarSafetyNode::fault_event_callback, this, std::placeholders::_1), subscription_options);

    RCLCPP_INFO(this->get_logger(), "LiDAR safety node started");
    RCLCPP_INFO(
      this->get_logger(), "Safety configuration: stop_distance=%.2f m, front_angle=%.1f deg",
      config_.stop_distance_m, config_.front_angle_deg);
  }

  [[nodiscard]] std::size_t executor_threads() const noexcept
  {
    return executor_threads_;
  }

private:
  /// Reject invalid scan-topic and front-ROI configuration before subscriptions start.
  void validate_configuration(const std::string & scan_topic) const
  {
    if (scan_topic.empty()) {
      throw std::invalid_argument("scan_topic must not be empty");
    }
    if (config_.stop_distance_m < 0.0F) {
      throw std::invalid_argument("stop_distance_m must be non-negative");
    }
    if (config_.front_angle_deg <= 0.0F || config_.front_angle_deg > 180.0F) {
      throw std::invalid_argument("front_angle_deg must be in (0, 180]");
    }
  }

  /// Evaluate one scan and publish its independent safety decision.
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    // This counter is independent from the scan result and can be updated atomically.
    scan_callback_count_.fetch_add(1U);
    const LidarScanData scan{
      message->angle_min,
      message->angle_max,
      message->angle_increment,
      message->range_min,
      message->range_max,
      message->ranges};
    const LidarSafetyResult result = core_.evaluate(scan, config_);

    std_msgs::msg::Bool obstacle_message;
    obstacle_message.data = result.obstacle_detected;
    obstacle_publisher_->publish(obstacle_message);

    std_msgs::msg::Float32 min_distance_message;
    min_distance_message.data = result.min_distance_m;
    min_distance_publisher_->publish(min_distance_message);

    arm_navi_safety_interfaces::msg::SafetyStatus safety_status;
    safety_status.header = message->header;
    safety_status.source = "lidar_safety";
    if (!result.has_valid_measurement) {
      safety_status.level = arm_navi_safety_interfaces::msg::SafetyStatus::ERROR;
      safety_status.reason = "no valid LiDAR measurement in front ROI";
      safety_status.data_valid = false;
    } else if (result.obstacle_detected) {
      safety_status.level = arm_navi_safety_interfaces::msg::SafetyStatus::STOP;
      safety_status.reason = "obstacle inside stop distance";
      safety_status.data_valid = true;
    } else {
      safety_status.level = arm_navi_safety_interfaces::msg::SafetyStatus::SAFE;
      safety_status.reason = "front ROI clear";
      safety_status.data_valid = true;
    }
    if (!result.has_valid_measurement) {
      const auto event = latest_latency_fault();
      if (event.has_value() && event->fault_type == "lidar_invalid_data") {
        // T1 is the first LiDAR safety callback that recognizes the injected invalid input.
        safety_status.latency_event_id = event->event_id;
        safety_status.latency_fault_type = event->fault_type;
        safety_status.fault_time_steady_ns = event->fault_time_steady_ns;
        safety_status.detection_time_steady_ns = steady_now_ns();
      }
    }
    safety_status_publisher_->publish(safety_status);

    bool obstacle_state_changed = false;
    {
      // Only protect the read-modify-write log state; publishing must not hold this mutex.
      std::lock_guard<std::mutex> lock(last_obstacle_mutex_);
      obstacle_state_changed = !last_obstacle_detected_.has_value() ||
        last_obstacle_detected_.value() != result.obstacle_detected;
      if (obstacle_state_changed) {
        last_obstacle_detected_ = result.obstacle_detected;
      }
    }
    if (obstacle_state_changed) {
      if (result.obstacle_detected) {
        RCLCPP_WARN(
          this->get_logger(), "Obstacle detected: min_distance=%.2f m", result.min_distance_m);
      } else if (result.has_valid_measurement) {
        RCLCPP_INFO(
          this->get_logger(), "Obstacle cleared: min_distance=%.2f m", result.min_distance_m);
      } else {
        RCLCPP_INFO(this->get_logger(), "No valid LiDAR point in the front ROI");
      }
    }
  }

  void fault_event_callback(const arm_navi_safety_interfaces::msg::SafetyEvent::SharedPtr message)
  {
    if (message->source != "lidar" || message->event_id == 0U ||
      message->fault_time_steady_ns <= 0 || message->fault_type.empty())
    {
      return;
    }
    std::lock_guard<std::mutex> lock(latency_fault_mutex_);
    latest_fault_event_ = LatencyFaultEvent{
      message->event_id, message->fault_type, message->fault_time_steady_ns};
  }

  [[nodiscard]] std::optional<LatencyFaultEvent> latest_latency_fault() const
  {
    std::lock_guard<std::mutex> lock(latency_fault_mutex_);
    return latest_fault_event_;
  }

  // Configuration and core are immutable after construction and safe for reentrant callbacks.
  LidarSafetyConfig config_{};
  LidarSafetyCore core_{};
  std::size_t executor_threads_{4U};
  // Reentrant because scan processing does not mutate decision state.
  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
  // Diagnostic-only counter; no compound state needs to be protected with it.
  std::atomic<std::uint64_t> scan_callback_count_{0U};
  // Guards the compound read/compare/write operation for transition logging.
  std::mutex last_obstacle_mutex_;
  std::optional<bool> last_obstacle_detected_{};
  mutable std::mutex latency_fault_mutex_;
  std::optional<LatencyFaultEvent> latest_fault_event_{};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_publisher_{};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr min_distance_publisher_{};
  rclcpp::Publisher<arm_navi_safety_interfaces::msg::SafetyStatus>::SharedPtr
    safety_status_publisher_{};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_{};
  rclcpp::Subscription<arm_navi_safety_interfaces::msg::SafetyEvent>::SharedPtr
    fault_event_subscription_{};
};

}  // namespace lidar_safety

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<lidar_safety::LidarSafetyNode>();
  // Keep scan processing responsive when multiple messages are ready.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), node->executor_threads());
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

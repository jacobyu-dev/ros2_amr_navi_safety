#include "safety_test_tools/fake_lidar.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "arm_navi_safety_interfaces/srv/set_fault_mode.hpp"
#include "arm_navi_safety_interfaces/msg/safety_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace safety_test_tools
{
namespace
{

std::chrono::milliseconds positive_milliseconds(
  rclcpp::Node & node, const std::string & name, std::int64_t default_value)
{
  const auto value = node.declare_parameter<std::int64_t>(name, default_value);
  if (value <= 0) {
    throw std::invalid_argument(name + " must be positive");
  }
  return std::chrono::milliseconds{value};
}

void validate_absolute_name(const std::string & value, const std::string & parameter_name)
{
  if (value.empty() || value.front() != '/') {
    throw std::invalid_argument(parameter_name + " must be an absolute ROS name");
  }
}

const char * latency_fault_type(FaultMode mode) noexcept
{
  switch (mode) {
    case FaultMode::STOP_PUBLISH:
    case FaultMode::SLOW_PUBLISH:
      return "lidar_timeout";
    case FaultMode::INVALID_DATA:
      return "lidar_invalid_data";
    case FaultMode::STALE_TIMESTAMP:
      return "lidar_stale_timestamp";
    case FaultMode::NORMAL:
      return "";
  }
  return "";
}

}  // namespace

/**
 * @brief Deterministic LaserScan producer for Phase 8 fault-injection tests.
 *
 * The timer runs at the normal publish cadence.  The service changes only an
 * atomic enum, allowing a mode request and a publish callback to overlap
 * without a compound shared state or blocking a callback.
 */
class FakeLidarNode : public rclcpp::Node
{
public:
  FakeLidarNode()
  : Node("fake_lidar_node"),
    normal_publish_period_(positive_milliseconds(*this, "normal_publish_period_ms", 100)),
    slow_publish_period_(positive_milliseconds(*this, "slow_publish_period_ms", 2000)),
    message_factory_(read_config()),
    last_slow_publish_ns_(steady_now_ns())
  {
    const auto scan_topic = this->declare_parameter<std::string>("scan_topic", "/scan");
    const auto service_name = this->declare_parameter<std::string>(
      "set_fault_mode_service", "/fake_lidar/set_fault_mode");
    const auto fault_event_topic = this->declare_parameter<std::string>(
      "fault_event_topic", "/safety/fault_injection/events");
    validate_absolute_name(scan_topic, "scan_topic");
    validate_absolute_name(service_name, "set_fault_mode_service");
    validate_absolute_name(fault_event_topic, "fault_event_topic");

    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS());
    fault_event_publisher_ = this->create_publisher<arm_navi_safety_interfaces::msg::SafetyEvent>(
      fault_event_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    service_ = this->create_service<arm_navi_safety_interfaces::srv::SetFaultMode>(
      service_name,
      std::bind(&FakeLidarNode::set_fault_mode, this, std::placeholders::_1, std::placeholders::_2));
    publish_timer_ = this->create_wall_timer(
      normal_publish_period_, std::bind(&FakeLidarNode::publish_scan, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Fake LiDAR started: mode=%s, normal=%ld ms, slow=%ld ms, topic=%s",
      to_string(FaultMode::NORMAL), normal_publish_period_.count(), slow_publish_period_.count(),
      scan_topic.c_str());
  }

private:
  [[nodiscard]] FakeLidarConfig read_config()
  {
    FakeLidarConfig config;
    config.normal_range_m = this->declare_parameter<float>("normal_range_m", 2.0F);
    config.range_min_m = this->declare_parameter<float>("range_min_m", 0.05F);
    config.range_max_m = this->declare_parameter<float>("range_max_m", 10.0F);
    config.stale_timestamp_offset = positive_milliseconds(*this, "stale_timestamp_offset_ms", 1000);
    return config;
  }

  static std::int64_t steady_now_ns() noexcept
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void set_fault_mode(
    const arm_navi_safety_interfaces::srv::SetFaultMode::Request::SharedPtr request,
    arm_navi_safety_interfaces::srv::SetFaultMode::Response::SharedPtr response)
  {
    FaultMode requested_mode{};
    if (!fault_mode_from_u8(request->mode, requested_mode)) {
      response->success = false;
      response->message = "unsupported fault mode " + std::to_string(request->mode);
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    const auto previous = mode_.exchange(requested_mode);
    if (requested_mode != FaultMode::NORMAL) {
      // T0 is the accepted fault-injection request, before the altered sensor behavior begins.
      const auto fault_time_ns = steady_now_ns();
      arm_navi_safety_interfaces::msg::SafetyEvent event;
      event.header.stamp = this->now();
      event.source = "lidar";
      event.previous_level = arm_navi_safety_interfaces::msg::SafetyEvent::SAFE;
      event.current_level = arm_navi_safety_interfaces::msg::SafetyEvent::ERROR;
      event.reason = "fake LiDAR fault injection";
      event.data_valid = false;
      event.event_id = next_event_id_.fetch_add(1U);
      event.fault_type = latency_fault_type(requested_mode);
      event.fault_time_steady_ns = fault_time_ns;
      fault_event_publisher_->publish(event);
    }
    if (requested_mode == FaultMode::SLOW_PUBLISH) {
      // Start the slow period at the explicit request, making the following
      // watchdog timeout deterministic rather than depending on the last scan.
      last_slow_publish_ns_.store(steady_now_ns());
    }
    response->success = true;
    response->message = "fault mode=" + std::string(to_string(requested_mode));
    if (previous != requested_mode) {
      RCLCPP_INFO(
        this->get_logger(), "Fake LiDAR fault mode changed: %s -> %s",
        to_string(previous), to_string(requested_mode));
    }
  }

  void publish_scan()
  {
    const auto mode = mode_.load();
    if (mode == FaultMode::STOP_PUBLISH) {
      return;
    }
    if (mode == FaultMode::SLOW_PUBLISH) {
      const auto now_ns = steady_now_ns();
      const auto last_ns = last_slow_publish_ns_.load();
      const auto slow_period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        slow_publish_period_).count();
      if (now_ns - last_ns < slow_period_ns) {
        return;
      }
      last_slow_publish_ns_.store(now_ns);
    }
    const builtin_interfaces::msg::Time stamp = this->now();
    publisher_->publish(message_factory_.make_scan(mode, stamp));
  }

  const std::chrono::milliseconds normal_publish_period_;
  const std::chrono::milliseconds slow_publish_period_;
  FakeLidarMessageFactory message_factory_;
  std::atomic<FaultMode> mode_{FaultMode::NORMAL};
  std::atomic<std::uint64_t> next_event_id_{1U};
  std::atomic<std::int64_t> last_slow_publish_ns_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Publisher<arm_navi_safety_interfaces::msg::SafetyEvent>::SharedPtr fault_event_publisher_;
  rclcpp::Service<arm_navi_safety_interfaces::srv::SetFaultMode>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace safety_test_tools

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<safety_test_tools::FakeLidarNode>());
  rclcpp::shutdown();
  return 0;
}

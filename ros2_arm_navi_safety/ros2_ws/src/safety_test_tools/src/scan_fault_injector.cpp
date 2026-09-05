#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "arm_navi_safety_interfaces/msg/safety_event.hpp"
#include "arm_navi_safety_interfaces/srv/set_fault_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "safety_test_tools/fake_lidar.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace safety_test_tools
{

class ScanFaultInjector : public rclcpp::Node
{
public:
  ScanFaultInjector()
  : Node("scan_fault_injector")
  {
    const auto input_topic = declare_parameter<std::string>("input_scan_topic", "/scan/raw");
    const auto output_topic = declare_parameter<std::string>("output_scan_topic", "/scan");
    const auto service_name = declare_parameter<std::string>(
      "set_fault_mode_service", "/fake_lidar/set_fault_mode");
    const auto event_topic = declare_parameter<std::string>(
      "fault_event_topic", "/safety/fault_injection/events");
    const auto stale_ms = declare_parameter<int>("stale_timestamp_offset_ms", 1000);
    const auto slow_ms = declare_parameter<int>("slow_publish_period_ms", 2000);
    if (input_topic.empty() || output_topic.empty() || service_name.empty() || event_topic.empty() ||
      input_topic.front() != '/' || output_topic.front() != '/' || service_name.front() != '/' ||
      event_topic.front() != '/' || input_topic == output_topic)
    {
      throw std::invalid_argument("scan fault injector requires distinct absolute ROS names");
    }
    if (stale_ms <= 0 || slow_ms <= 0) {
      throw std::invalid_argument("scan fault injector timing parameters must be positive");
    }
    stale_offset_ = std::chrono::milliseconds(stale_ms);
    slow_period_ = std::chrono::milliseconds(slow_ms);
    output_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic, rclcpp::SensorDataQoS());
    events_ = create_publisher<arm_navi_safety_interfaces::msg::SafetyEvent>(
      event_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    input_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&ScanFaultInjector::scan_callback, this, std::placeholders::_1));
    service_ = create_service<arm_navi_safety_interfaces::srv::SetFaultMode>(
      service_name,
      std::bind(&ScanFaultInjector::set_mode, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(
      get_logger(), "Gazebo scan fault relay: %s -> %s, service=%s", input_topic.c_str(),
      output_topic.c_str(), service_name.c_str());
  }

private:
  static std::int64_t steady_now_ns() noexcept
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void set_mode(
    const arm_navi_safety_interfaces::srv::SetFaultMode::Request::SharedPtr request,
    arm_navi_safety_interfaces::srv::SetFaultMode::Response::SharedPtr response)
  {
    FaultMode requested{};
    if (!fault_mode_from_u8(request->mode, requested)) {
      response->success = false;
      response->message = "unsupported fault mode " + std::to_string(request->mode);
      return;
    }
    const auto previous = mode_.exchange(requested);
    last_slow_publish_ns_.store(steady_now_ns());
    if (requested != FaultMode::NORMAL) {
      arm_navi_safety_interfaces::msg::SafetyEvent event;
      event.header.stamp = now();
      event.source = "lidar";
      event.previous_level = arm_navi_safety_interfaces::msg::SafetyEvent::SAFE;
      event.current_level = arm_navi_safety_interfaces::msg::SafetyEvent::ERROR;
      event.reason = "Gazebo scan relay fault injection";
      event.data_valid = false;
      event.event_id = next_event_id_.fetch_add(1U);
      event.fault_type = requested == FaultMode::INVALID_DATA ? "lidar_invalid_data" :
        requested == FaultMode::STALE_TIMESTAMP ? "lidar_stale_timestamp" : "lidar_timeout";
      event.fault_time_steady_ns = steady_now_ns();
      events_->publish(event);
    }
    response->success = true;
    response->message = "fault mode=" + std::string(to_string(requested));
    RCLCPP_INFO(
      get_logger(), "Gazebo scan fault mode: %s -> %s", to_string(previous), to_string(requested));
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr input)
  {
    const auto mode = mode_.load();
    if (mode == FaultMode::STOP_PUBLISH) {
      return;
    }
    if (mode == FaultMode::SLOW_PUBLISH) {
      const auto current = steady_now_ns();
      const auto slow_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(slow_period_).count();
      if (current - last_slow_publish_ns_.load() < slow_ns) {
        return;
      }
      last_slow_publish_ns_.store(current);
    }
    auto output = *input;
    if (mode == FaultMode::INVALID_DATA) {
      for (auto & range : output.ranges) {
        range = std::numeric_limits<float>::quiet_NaN();
      }
    } else if (mode == FaultMode::STALE_TIMESTAMP) {
      const auto stamp = rclcpp::Time(output.header.stamp);
      const auto offset = rclcpp::Duration::from_nanoseconds(
        std::chrono::duration_cast<std::chrono::nanoseconds>(stale_offset_).count());
      output.header.stamp = stamp > rclcpp::Time(offset.nanoseconds()) ? stamp - offset :
        rclcpp::Time(0, 0, stamp.get_clock_type());
    }
    output_->publish(output);
  }

  std::atomic<FaultMode> mode_{FaultMode::NORMAL};
  std::atomic<std::uint64_t> next_event_id_{1U};
  std::atomic<std::int64_t> last_slow_publish_ns_{0};
  std::chrono::milliseconds stale_offset_{1000};
  std::chrono::milliseconds slow_period_{2000};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr input_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr output_;
  rclcpp::Publisher<arm_navi_safety_interfaces::msg::SafetyEvent>::SharedPtr events_;
  rclcpp::Service<arm_navi_safety_interfaces::srv::SetFaultMode>::SharedPtr service_;
};

}  // namespace safety_test_tools

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<safety_test_tools::ScanFaultInjector>());
  rclcpp::shutdown();
  return 0;
}

#ifndef TF_MONITOR__TF_MONITOR_NODE_HPP_
#define TF_MONITOR__TF_MONITOR_NODE_HPP_

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_set>

#include "arm_navi_safety_interfaces/msg/safety_status.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf_monitor/tf_health_checker.hpp"

namespace tf_monitor
{

/**
 * @brief Monitors one TF relation and publishes its safety health status.
 *
 * Static-TF messages may update while the health timer reads the configured
 * pair, so static_frame_pairs_ is protected by a mutex. The timer's group is
 * mutually exclusive to prevent overlapping TF lookups and status transitions.
 */
class TfMonitorNode : public rclcpp::Node
{
public:
  /// Declare TF configuration, create I/O, and assign callback groups.
  TfMonitorNode();

  /// Return the validated thread count used by this executable's MTE.
  [[nodiscard]] std::size_t executor_threads() const noexcept;

private:
  /// Look up the configured relation and publish its current health.
  void check_transform();

  /// Record directly observed static frame pairs for static-transform health checks.
  void static_tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr message);

  /// Publish Bool, diagnostics, and SafetyStatus messages for one evaluated TF result.
  void publish_status(TfStatus status, std::chrono::milliseconds age, bool is_static);

  /// Check whether the configured direct relation was received on /tf_static.
  [[nodiscard]] bool is_direct_static_transform() const;

  /// Create the stable lookup key shared by static callback and timer.
  [[nodiscard]] std::string frame_pair_key() const;

  std::string parent_frame_;
  std::string child_frame_;
  std::size_t executor_threads_{4U};
  TfHealthChecker health_checker_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr healthy_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<arm_navi_safety_interfaces::msg::SafetyStatus>::SharedPtr
    safety_status_publisher_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_tf_subscription_;
  // Static updates may run concurrently with monitoring.
  rclcpp::CallbackGroup::SharedPtr static_tf_callback_group_;
  // Prevent overlapping timer checks and writes to last_status_.
  rclcpp::CallbackGroup::SharedPtr monitor_callback_group_;
  // Protects the static-pair set across the reentrant subscription and timer.
  mutable std::mutex static_frame_pairs_mutex_;
  std::unordered_set<std::string> static_frame_pairs_;
  // Independent diagnostic counter updated by the reentrant callback.
  std::atomic<std::uint64_t> static_tf_callback_count_{0U};
  // This is touched only by monitor_callback_group_, which is mutually exclusive.
  std::optional<TfStatus> last_status_;
};

}  // namespace tf_monitor

#endif  // TF_MONITOR__TF_MONITOR_NODE_HPP_

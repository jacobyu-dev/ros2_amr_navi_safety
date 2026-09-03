#include "tf_monitor/tf_monitor_node.hpp"

#include <chrono>
#include <functional>
#include <stdexcept>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"

namespace tf_monitor
{
namespace
{
using namespace std::chrono_literals;

// Declare all timing parameters before constructing the immutable health checker.
TfMonitorConfig read_config(rclcpp::Node & node)
{
  const auto check_period_ms = node.declare_parameter<std::int64_t>("check_period_ms", 100);
  const auto lookup_timeout_ms = node.declare_parameter<std::int64_t>("lookup_timeout_ms", 100);
  const auto stale_threshold_ms = node.declare_parameter<std::int64_t>("stale_threshold_ms", 500);
  const bool allow_static_transform = node.declare_parameter<bool>("allow_static_transform", true);
  return TfMonitorConfig{
    std::chrono::milliseconds{check_period_ms},
    std::chrono::milliseconds{lookup_timeout_ms},
    std::chrono::milliseconds{stale_threshold_ms},
    allow_static_transform};
}

// Construct one diagnostic key-value item while keeping publish_status readable.
diagnostic_msgs::msg::KeyValue value(std::string key, std::string text)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(text);
  return result;
}
}  // namespace

TfMonitorNode::TfMonitorNode()
: Node("tf_monitor_node"),
  parent_frame_(this->declare_parameter<std::string>("parent_frame", "base_link")),
  child_frame_(this->declare_parameter<std::string>("child_frame", "laser_frame")),
  health_checker_(read_config(*this))
{
  if (parent_frame_.empty() || child_frame_.empty()) {
    throw std::invalid_argument("parent_frame and child_frame must not be empty");
  }
  if (parent_frame_ == child_frame_) {
    throw std::invalid_argument("parent_frame and child_frame must be different");
  }

  const std::string output_prefix = this->declare_parameter<std::string>(
    "output_prefix", "/tf_monitor");
  const std::string safety_status_topic = this->declare_parameter<std::string>(
    "safety_status_topic", "/safety/tf/status");
  const auto configured_executor_threads = this->declare_parameter<int>("executor_threads", 4);
  if (output_prefix.empty() || output_prefix.front() != '/') {
    throw std::invalid_argument("output_prefix must be an absolute ROS topic prefix");
  }
  if (safety_status_topic.empty() || safety_status_topic.front() != '/') {
    throw std::invalid_argument("safety_status_topic must be an absolute ROS topic");
  }
  if (configured_executor_threads <= 0) {
    throw std::invalid_argument("executor_threads must be positive");
  }
  executor_threads_ = static_cast<std::size_t>(configured_executor_threads);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  // A dedicated TF listener thread is required by TF2 when lookupTransform uses
  // a non-zero timeout. This is TF2's listener thread, not an application executor.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  healthy_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
    output_prefix + "/healthy", rclcpp::QoS(10));
  diagnostics_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    output_prefix + "/diagnostics", rclcpp::QoS(10));
  safety_status_publisher_ = this->create_publisher<arm_navi_safety_interfaces::msg::SafetyStatus>(
    safety_status_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  // Static TF input can be received while the health timer runs. Its aggregate
  // is copied under a mutex; the timer remains serial to protect last_status_.
  static_tf_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  monitor_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions static_tf_subscription_options;
  static_tf_subscription_options.callback_group = static_tf_callback_group_;
  static_tf_subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&TfMonitorNode::static_tf_callback, this, std::placeholders::_1),
    static_tf_subscription_options);
  check_timer_ = this->create_wall_timer(
    health_checker_.config().check_period,
    std::bind(&TfMonitorNode::check_transform, this), monitor_callback_group_);

  RCLCPP_INFO(
    this->get_logger(), "TF monitor started: %s -> %s, period=%ld ms, stale=%ld ms",
    parent_frame_.c_str(), child_frame_.c_str(),
    health_checker_.config().check_period.count(), health_checker_.config().stale_threshold.count());
}

void TfMonitorNode::check_transform()
{
  // This runs in the mutually exclusive monitor group, so last_status_ cannot race.
  try {
    const auto transform = tf_buffer_->lookupTransform(
      parent_frame_, child_frame_, tf2::TimePointZero, health_checker_.config().lookup_timeout);
    const rclcpp::Time stamp(transform.header.stamp, this->get_clock()->get_clock_type());
    const auto now = this->now();
    const auto age = now >= stamp ? std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::nanoseconds{(now - stamp).nanoseconds()}) : 0ms;
    const bool is_static = is_direct_static_transform();
    const TfStatus status = health_checker_.evaluate(TfObservation{true, is_static, age});
    publish_status(status, age, is_static);
  } catch (const tf2::TimeoutException &) {
    publish_status(status_from_lookup_failure(TfLookupFailure::TIMEOUT), 0ms, false);
  } catch (const tf2::ExtrapolationException &) {
    publish_status(status_from_lookup_failure(TfLookupFailure::TIMEOUT), 0ms, false);
  } catch (const tf2::LookupException &) {
    publish_status(status_from_lookup_failure(TfLookupFailure::MISSING), 0ms, false);
  } catch (const tf2::ConnectivityException &) {
    publish_status(status_from_lookup_failure(TfLookupFailure::MISSING), 0ms, false);
  } catch (const tf2::TransformException &) {
    publish_status(status_from_lookup_failure(TfLookupFailure::MISSING), 0ms, false);
  }
}

void TfMonitorNode::static_tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr message)
{
  static_tf_callback_count_.fetch_add(1U);
  // Keep insertion atomic with respect to is_direct_static_transform().
  std::lock_guard<std::mutex> lock(static_frame_pairs_mutex_);
  for (const auto & transform : message->transforms) {
    static_frame_pairs_.insert(transform.header.frame_id + "\n" + transform.child_frame_id);
  }
}

void TfMonitorNode::publish_status(TfStatus status, std::chrono::milliseconds age, bool is_static)
{
  // Called only by the serialized monitor timer; publishing occurs without the static-pair mutex.
  std_msgs::msg::Bool healthy_message;
  healthy_message.data = status == TfStatus::OK;
  healthy_publisher_->publish(healthy_message);

  diagnostic_msgs::msg::DiagnosticStatus diagnostic;
  diagnostic.name = "tf_monitor/" + parent_frame_ + "_to_" + child_frame_;
  diagnostic.hardware_id = "tf2";
  diagnostic.level = status == TfStatus::OK ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  diagnostic.message = to_string(status);
  diagnostic.values.push_back(value("parent_frame", parent_frame_));
  diagnostic.values.push_back(value("child_frame", child_frame_));
  diagnostic.values.push_back(value("status", to_string(status)));
  diagnostic.values.push_back(value("transform_age_ms", std::to_string(age.count())));
  diagnostic.values.push_back(value(
    "stale_threshold_ms", std::to_string(health_checker_.config().stale_threshold.count())));
  diagnostic.values.push_back(value("static_transform", is_static ? "true" : "false"));

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = this->now();
  array.status.push_back(std::move(diagnostic));
  diagnostics_publisher_->publish(array);

  arm_navi_safety_interfaces::msg::SafetyStatus safety_status;
  safety_status.header.stamp = this->now();
  safety_status.header.frame_id = parent_frame_;
  safety_status.source = "tf_monitor";
  safety_status.reason = "TF " + std::string(to_string(status));
  safety_status.level = status == TfStatus::OK ?
    arm_navi_safety_interfaces::msg::SafetyStatus::SAFE :
    arm_navi_safety_interfaces::msg::SafetyStatus::ERROR;
  safety_status.data_valid = status == TfStatus::OK;
  safety_status_publisher_->publish(safety_status);

  if (!last_status_.has_value() || last_status_.value() != status) {
    if (status == TfStatus::OK) {
      RCLCPP_INFO(this->get_logger(), "TF %s -> %s OK", parent_frame_.c_str(), child_frame_.c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(), "TF %s -> %s %s", parent_frame_.c_str(), child_frame_.c_str(),
        to_string(status));
    }
    last_status_ = status;
  }
}

bool TfMonitorNode::is_direct_static_transform() const
{
  if (!health_checker_.config().allow_static_transform) {
    return false;
  }
  // Only the set lookup needs synchronization; the resulting bool is a local value.
  std::lock_guard<std::mutex> lock(static_frame_pairs_mutex_);
  return static_frame_pairs_.count(frame_pair_key()) != 0U;
}

std::string TfMonitorNode::frame_pair_key() const
{
  return parent_frame_ + "\n" + child_frame_;
}

std::size_t TfMonitorNode::executor_threads() const noexcept
{
  return executor_threads_;
}

}  // namespace tf_monitor

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<tf_monitor::TfMonitorNode>();
  // The callback groups inside TfMonitorNode define which work may overlap.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), node->executor_threads());
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

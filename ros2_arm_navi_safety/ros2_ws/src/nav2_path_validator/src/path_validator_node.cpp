#include "nav2_path_validator/node_factory.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "arm_navi_safety_interfaces/msg/mission_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_path_validator/path_metrics.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_path_validator
{
namespace
{

constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

enum class ValidationState
{
  WAITING_FOR_NAVIGATION,
  COLLECTING_PATH,
  TRACKING,
  GOAL_REACHED,
  VALIDATING,
  PASSED,
  FAILED
};

const char * toString(ValidationState state) noexcept
{
  switch (state) {
    case ValidationState::WAITING_FOR_NAVIGATION: return "WAITING_FOR_NAVIGATION";
    case ValidationState::COLLECTING_PATH: return "COLLECTING_PATH";
    case ValidationState::TRACKING: return "TRACKING";
    case ValidationState::GOAL_REACHED: return "GOAL_REACHED";
    case ValidationState::VALIDATING: return "VALIDATING";
    case ValidationState::PASSED: return "PASSED";
    case ValidationState::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

struct PoseSample
{
  double stamp_seconds{0.0};
  Point2D point;
  double yaw{0.0};
  double tracking_error{std::numeric_limits<double>::quiet_NaN()};
};

std::vector<Point2D> pointsFromPath(const nav_msgs::msg::Path & message)
{
  std::vector<Point2D> result;
  result.reserve(message.poses.size());
  for (const auto & pose : message.poses) {
    result.push_back(Point2D{pose.pose.position.x, pose.pose.position.y});
  }
  return result;
}

std::string jsonEscape(const std::string & input)
{
  std::ostringstream output;
  for (const char character : input) {
    switch (character) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default: output << character; break;
    }
  }
  return output.str();
}

std::string timestampForFilename()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
#ifdef _WIN32
  localtime_s(&local_time, &time);
#else
  localtime_r(&time, &local_time);
#endif
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stream.str();
}

void requirePositive(double value, const std::string & name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(name + " must be finite and positive");
  }
}

}  // namespace

class PathValidatorNode : public rclcpp::Node
{
public:
  explicit PathValidatorNode(const rclcpp::NodeOptions & options)
  : Node("nav2_path_validator", options),
    tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    global_path_topic_ = declare_parameter<std::string>("global_path_topic", "/plan");
    local_path_topic_ = declare_parameter<std::string>(
      "local_path_topic", "/received_global_plan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/mission/active_goal");
    mission_status_topic_ = declare_parameter<std::string>("mission_status_topic",
        "/mission/status");
    action_status_topic_ = declare_parameter<std::string>(
      "action_status_topic", "/navigate_to_pose/_action/status");
    validation_frame_ = declare_parameter<std::string>("validation_frame", "map");
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");
    pose_source_ = declare_parameter<std::string>("pose_source", "tf");
    completion_source_ = declare_parameter<std::string>("completion_source", "mission_status");
    scenario_ = declare_parameter<std::string>("scenario", "observer");
    results_directory_ = declare_parameter<std::string>("results_directory", "results");
    trajectory_sample_distance_ = declare_parameter<double>("trajectory_sample_distance", 0.02);
    sample_period_seconds_ = declare_parameter<double>("trajectory_sample_period", 0.05);
    path_goal_tolerance_ = declare_parameter<double>("path_goal_tolerance", 0.25);
    goal_position_tolerance_ = declare_parameter<double>("goal_position_tolerance", 0.25);
    goal_yaw_tolerance_deg_ = declare_parameter<double>("goal_yaw_tolerance_deg", 15.0);
    max_tracking_rmse_ = declare_parameter<double>("max_tracking_rmse", 0.30);
    max_tracking_error_ = declare_parameter<double>("max_tracking_error", 0.70);
    replan_change_threshold_ = declare_parameter<double>("replan_change_threshold", 0.20);
    write_csv_ = declare_parameter<bool>("write_csv", true);

    if (validation_frame_.empty() || robot_base_frame_.empty()) {
      throw std::invalid_argument("validation_frame and robot_base_frame must not be empty");
    }
    if (pose_source_ != "tf" && pose_source_ != "odom") {
      throw std::invalid_argument("pose_source must be 'tf' or 'odom'");
    }
    if (completion_source_ != "mission_status" && completion_source_ != "action_status" &&
      completion_source_ != "manual")
    {
      throw std::invalid_argument(
              "completion_source must be 'mission_status', 'action_status', or 'manual'");
    }
    requirePositive(trajectory_sample_distance_, "trajectory_sample_distance");
    requirePositive(sample_period_seconds_, "trajectory_sample_period");
    requirePositive(path_goal_tolerance_, "path_goal_tolerance");
    requirePositive(goal_position_tolerance_, "goal_position_tolerance");
    requirePositive(goal_yaw_tolerance_deg_, "goal_yaw_tolerance_deg");
    requirePositive(max_tracking_rmse_, "max_tracking_rmse");
    requirePositive(max_tracking_error_, "max_tracking_error");
    requirePositive(replan_change_threshold_, "replan_change_threshold");

    const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    global_path_subscription_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic_, path_qos,
      std::bind(&PathValidatorNode::onGlobalPath, this, std::placeholders::_1));
    local_path_subscription_ = create_subscription<nav_msgs::msg::Path>(
      local_path_topic_, path_qos,
      std::bind(&PathValidatorNode::onLocalPath, this, std::placeholders::_1));
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&PathValidatorNode::onGoal, this, std::placeholders::_1));

    if (pose_source_ == "odom") {
      odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PathValidatorNode::onOdometry, this, std::placeholders::_1));
    } else {
      sample_timer_ = create_wall_timer(
        std::chrono::duration<double>(sample_period_seconds_),
        std::bind(&PathValidatorNode::sampleTransform, this));
    }

    if (completion_source_ == "mission_status") {
      mission_status_subscription_ =
        create_subscription<arm_navi_safety_interfaces::msg::MissionStatus>(
        mission_status_topic_, rclcpp::QoS(20).reliable(),
        std::bind(&PathValidatorNode::onMissionStatus, this, std::placeholders::_1));
    } else if (completion_source_ == "action_status") {
      action_status_subscription_ = create_subscription<action_msgs::msg::GoalStatusArray>(
        action_status_topic_, rclcpp::QoS(20).reliable(),
        std::bind(&PathValidatorNode::onActionStatus, this, std::placeholders::_1));
    }

    trajectory_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/nav2_path_validation/actual_trajectory",
      rclcpp::QoS(1).reliable().transient_local());
    state_publisher_ = create_publisher<std_msgs::msg::String>(
      "/nav2_path_validation/state", rclcpp::QoS(1).reliable().transient_local());
    result_publisher_ = create_publisher<std_msgs::msg::String>(
      "/nav2_path_validation/result", rclcpp::QoS(1).reliable().transient_local());
    finalize_service_ = create_service<std_srvs::srv::Trigger>(
      "/nav2_path_validation/finalize_success",
      std::bind(&PathValidatorNode::onFinalize, this, std::placeholders::_1,
        std::placeholders::_2));

    publishState();
    RCLCPP_INFO(
      get_logger(),
      "Path validator ready: global_path=%s local_path=%s pose_source=%s frame=%s completion=%s",
      global_path_topic_.c_str(), local_path_topic_.c_str(), pose_source_.c_str(),
      validation_frame_.c_str(), completion_source_.c_str());
  }

private:
  void setState(ValidationState state)
  {
    if (state_ == state) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Validation state: %s -> %s", toString(state_), toString(state));
    state_ = state;
    publishState();
  }

  void publishState()
  {
    if (!state_publisher_) {
      return;
    }
    std_msgs::msg::String message;
    message.data = toString(state_);
    state_publisher_->publish(message);
  }

  bool transformGoal(
    const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output)
  {
    if (input.header.frame_id == validation_frame_) {
      output = input;
      return true;
    }
    try {
      output = tf_buffer_.transform(input, validation_frame_, tf2::durationFromSec(0.2));
      return true;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN(
        get_logger(), "Cannot transform goal from %s to %s: %s", input.header.frame_id.c_str(),
        validation_frame_.c_str(), exception.what());
      return false;
    }
  }

  void resetForGoal(const geometry_msgs::msg::PoseStamped & goal)
  {
    goal_ = goal;
    have_goal_ = true;
    active_session_ = true;
    finalized_ = false;
    navigation_accepted_ = false;
    navigation_succeeded_ = false;
    path_received_ = false;
    path_sufficient_ = false;
    path_frame_valid_ = false;
    local_path_received_ = false;
    initial_path_.clear();
    current_path_.clear();
    trajectory_.clear();
    tracking_errors_.clear();
    initial_path_length_ = 0.0;
    latest_path_length_ = 0.0;
    path_start_error_ = std::numeric_limits<double>::quiet_NaN();
    path_goal_error_ = std::numeric_limits<double>::quiet_NaN();
    replan_count_ = 0U;
    global_path_messages_ = 0U;
    local_path_messages_ = 0U;
    session_started_ = now();
    setState(ValidationState::COLLECTING_PATH);
    RCLCPP_INFO(
      get_logger(), "Validation session started for goal (%.3f, %.3f) in %s",
      goal.pose.position.x, goal.pose.position.y, validation_frame_.c_str());
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    geometry_msgs::msg::PoseStamped transformed;
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring goal with an empty frame_id");
      return;
    }
    if (transformGoal(*message, transformed)) {
      resetForGoal(transformed);
    }
  }

  void onGlobalPath(const nav_msgs::msg::Path::SharedPtr message)
  {
    if (!active_session_) {
      RCLCPP_DEBUG(get_logger(), "Ignoring global path before a requested goal");
      return;
    }
    ++global_path_messages_;
    path_received_ = true;
    const bool valid_frame = message->header.frame_id == validation_frame_;
    const auto candidate = pointsFromPath(*message);
    const bool sufficient = candidate.size() > 1U;
    if (!valid_frame) {
      RCLCPP_ERROR(
        get_logger(), "Global path frame '%s' does not match validation frame '%s'",
        message->header.frame_id.c_str(), validation_frame_.c_str());
      path_frame_valid_ = false;
      return;
    }
    path_frame_valid_ = true;
    if (!sufficient) {
      RCLCPP_ERROR(get_logger(), "Global path has %zu pose(s); at least 2 are required",
          candidate.size());
      path_sufficient_ = false;
      return;
    }
    path_sufficient_ = true;

    if (initial_path_.empty()) {
      initial_path_ = candidate;
      initial_path_length_ = pathLength(initial_path_);
      if (!trajectory_.empty()) {
        path_start_error_ = distance(initial_path_.front(), trajectory_.front().point);
      }
      RCLCPP_INFO(
        get_logger(), "Global path received: frame=%s poses=%zu length=%.3f m",
        validation_frame_.c_str(), candidate.size(), initial_path_length_);
    } else {
      const double deviation = pathGeometryDeviation(current_path_, candidate);
      if (deviation > replan_change_threshold_) {
        ++replan_count_;
        RCLCPP_INFO(
          get_logger(),
            "Meaningful global replan detected: deviation=%.3f m threshold=%.3f m count=%zu",
          deviation, replan_change_threshold_, replan_count_);
      }
    }
    current_path_ = candidate;
    latest_path_length_ = pathLength(current_path_);
    path_goal_error_ = distance(
      current_path_.back(), Point2D{goal_.pose.position.x, goal_.pose.position.y});
    if (navigation_accepted_) {
      setState(ValidationState::TRACKING);
    }
  }

  void onLocalPath(const nav_msgs::msg::Path::SharedPtr message)
  {
    if (!active_session_) {
      return;
    }
    ++local_path_messages_;
    if (message->poses.size() > 1U) {
      local_path_received_ = true;
    }
  }

  void onMissionStatus(
    const arm_navi_safety_interfaces::msg::MissionStatus::SharedPtr message)
  {
    using Status = arm_navi_safety_interfaces::msg::MissionStatus;
    if (!active_session_ || finalized_) {
      return;
    }
    if (message->navigation_state == Status::NAVIGATING ||
      message->navigation_state == Status::NAVIGATION_PAUSED ||
      message->navigation_state == Status::ARRIVED)
    {
      navigation_accepted_ = true;
      if (path_sufficient_ && message->navigation_state == Status::NAVIGATING) {
        setState(ValidationState::TRACKING);
      }
    }
    if (message->mission_state == Status::COMPLETED) {
      navigation_succeeded_ = true;
      setState(ValidationState::GOAL_REACHED);
      finalize();
    } else if (message->mission_state == Status::FAILED ||
      message->mission_state == Status::CANCELED)
    {
      navigation_succeeded_ = false;
      finalize();
    }
  }

  void onActionStatus(const action_msgs::msg::GoalStatusArray::SharedPtr message)
  {
    if (!active_session_ || finalized_) {
      return;
    }
    const action_msgs::msg::GoalStatus * latest = nullptr;
    rclcpp::Time latest_stamp(0, 0, get_clock()->get_clock_type());
    for (const auto & status : message->status_list) {
      const rclcpp::Time stamp(status.goal_info.stamp, get_clock()->get_clock_type());
      if (stamp >= session_started_ && (!latest || stamp >= latest_stamp)) {
        latest = &status;
        latest_stamp = stamp;
      }
    }
    if (!latest) {
      return;
    }
    using GoalStatus = action_msgs::msg::GoalStatus;
    if (latest->status == GoalStatus::STATUS_ACCEPTED ||
      latest->status == GoalStatus::STATUS_EXECUTING ||
      latest->status == GoalStatus::STATUS_CANCELING)
    {
      navigation_accepted_ = true;
      if (path_sufficient_) {
        setState(ValidationState::TRACKING);
      }
    } else if (latest->status == GoalStatus::STATUS_SUCCEEDED) {
      navigation_accepted_ = true;
      navigation_succeeded_ = true;
      setState(ValidationState::GOAL_REACHED);
      finalize();
    } else if (latest->status == GoalStatus::STATUS_ABORTED ||
      latest->status == GoalStatus::STATUS_CANCELED)
    {
      navigation_succeeded_ = false;
      finalize();
    }
  }

  void sampleTransform()
  {
    if (!active_session_ || finalized_) {
      return;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        validation_frame_, robot_base_frame_, tf2::TimePointZero);
      observePose(
        Point2D{transform.transform.translation.x, transform.transform.translation.y},
        tf2::getYaw(transform.transform.rotation), now().seconds());
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for %s -> %s TF: %s",
        validation_frame_.c_str(), robot_base_frame_.c_str(), exception.what());
    }
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!active_session_ || finalized_) {
      return;
    }
    geometry_msgs::msg::PoseStamped source;
    source.header = message->header;
    source.pose = message->pose.pose;
    geometry_msgs::msg::PoseStamped transformed;
    if (source.header.frame_id == validation_frame_) {
      transformed = source;
    } else {
      try {
        transformed = tf_buffer_.transform(source, validation_frame_, tf2::durationFromSec(0.1));
      } catch (const tf2::TransformException & exception) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Cannot transform odometry %s -> %s: %s",
          source.header.frame_id.c_str(), validation_frame_.c_str(), exception.what());
        return;
      }
    }
    observePose(
      Point2D{transformed.pose.position.x, transformed.pose.position.y},
      tf2::getYaw(transformed.pose.orientation), rclcpp::Time(message->header.stamp).seconds());
  }

  void observePose(Point2D point, double yaw, double stamp_seconds)
  {
    if (!trajectory_.empty() &&
      distance(trajectory_.back().point, point) < trajectory_sample_distance_)
    {
      // Keep final orientation current without inflating path length or sample count.
      trajectory_.back().yaw = yaw;
      trajectory_.back().stamp_seconds = stamp_seconds;
      return;
    }
    PoseSample sample;
    sample.stamp_seconds = stamp_seconds;
    sample.point = point;
    sample.yaw = yaw;
    if (current_path_.size() > 1U) {
      sample.tracking_error = pointToPathDistance(point, current_path_);
      tracking_errors_.push_back(sample.tracking_error);
    }
    trajectory_.push_back(sample);
    if (!initial_path_.empty() && !std::isfinite(path_start_error_)) {
      path_start_error_ = distance(initial_path_.front(), trajectory_.front().point);
    }
    publishTrajectory();
  }

  void publishTrajectory()
  {
    nav_msgs::msg::Path message;
    message.header.stamp = now();
    message.header.frame_id = validation_frame_;
    message.poses.reserve(trajectory_.size());
    for (const auto & sample : trajectory_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = sample.point.x;
      pose.pose.position.y = sample.point.y;
      pose.pose.orientation.z = std::sin(sample.yaw * 0.5);
      pose.pose.orientation.w = std::cos(sample.yaw * 0.5);
      message.poses.push_back(std::move(pose));
    }
    trajectory_publisher_->publish(message);
  }

  void onFinalize(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!active_session_) {
      response->success = false;
      response->message = "No active validation session; publish a goal first";
      return;
    }
    navigation_accepted_ = true;
    navigation_succeeded_ = true;
    finalize();
    response->success = state_ == ValidationState::PASSED;
    response->message = response->success ? "PASS" : "FAIL; inspect /nav2_path_validation/result";
  }

  void finalize()
  {
    if (finalized_) {
      return;
    }

    // Completion status and the periodic TF sampler are independent callbacks.
    // Capture the latest pose here so the final-error check cannot use a stale
    // sample from just before Nav2 reported success.
    if (pose_source_ == "tf") {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          validation_frame_, robot_base_frame_, tf2::TimePointZero);
        observePose(
          Point2D{transform.transform.translation.x, transform.transform.translation.y},
          tf2::getYaw(transform.transform.rotation), now().seconds());
      } catch (const tf2::TransformException & exception) {
        RCLCPP_WARN(
          get_logger(), "Could not capture final %s -> %s TF; using last trajectory sample: %s",
          validation_frame_.c_str(), robot_base_frame_.c_str(), exception.what());
      }
    }

    finalized_ = true;
    setState(ValidationState::VALIDATING);
    const auto tracking = errorStatistics(tracking_errors_);
    const double travel_distance = trajectoryDistance();
    const bool have_final_pose = !trajectory_.empty();
    const double goal_position_error = have_final_pose ? distance(
      trajectory_.back().point, Point2D{goal_.pose.position.x, goal_.pose.position.y}) :
      std::numeric_limits<double>::infinity();
    const double goal_yaw_error = have_final_pose ? yawError(
      trajectory_.back().yaw, tf2::getYaw(goal_.pose.orientation)) :
      std::numeric_limits<double>::infinity();
    const bool passed = navigation_accepted_ && navigation_succeeded_ && have_goal_ &&
      path_received_ && path_sufficient_ && path_frame_valid_ &&
      std::isfinite(path_goal_error_) && path_goal_error_ <= path_goal_tolerance_ &&
      trajectory_.size() > 1U && tracking.valid && tracking.rmse <= max_tracking_rmse_ &&
      tracking.maximum <= max_tracking_error_ && goal_position_error <= goal_position_tolerance_ &&
      goal_yaw_error * kRadiansToDegrees <= goal_yaw_tolerance_deg_;

    const std::string json = resultJson(
      passed, tracking, travel_distance, goal_position_error,
      goal_yaw_error * kRadiansToDegrees);
    std_msgs::msg::String result_message;
    result_message.data = json;
    result_publisher_->publish(result_message);
    printSummary(
      passed, tracking, travel_distance, goal_position_error,
      goal_yaw_error * kRadiansToDegrees);
    if (write_csv_) {
      writeResults(
        passed, tracking, travel_distance, goal_position_error,
        goal_yaw_error * kRadiansToDegrees);
    }
    setState(passed ? ValidationState::PASSED : ValidationState::FAILED);
    active_session_ = false;
  }

  double trajectoryDistance() const
  {
    std::vector<Point2D> points;
    points.reserve(trajectory_.size());
    for (const auto & sample : trajectory_) {
      points.push_back(sample.point);
    }
    return pathLength(points);
  }

  std::string resultJson(
    bool passed, const ErrorStatistics & tracking, double travel_distance,
    double goal_position_error, double goal_yaw_error_deg) const
  {
    const double efficiency = initial_path_length_ > 0.0 ?
      travel_distance / initial_path_length_ : 0.0;
    std::ostringstream stream;
    stream << std::boolalpha << std::fixed << std::setprecision(6)
           << "{\"result\":\"" << (passed ? "PASS" : "FAIL") << "\""
           << ",\"scenario\":\"" << jsonEscape(scenario_) << "\""
           << ",\"frame\":\"" << jsonEscape(validation_frame_) << "\""
           << ",\"navigation_accepted\":" << navigation_accepted_
           << ",\"navigation_succeeded\":" << navigation_succeeded_
           << ",\"global_path_received\":" << path_received_
           << ",\"path_sufficient\":" << path_sufficient_
           << ",\"path_frame_valid\":" << path_frame_valid_
           << ",\"local_path_received\":" << local_path_received_
           << ",\"path_pose_count\":" << initial_path_.size()
           << ",\"global_path_length\":" << initial_path_length_
           << ",\"latest_path_length\":" << latest_path_length_
           << ",\"path_start_error\":" << finiteOrMinusOne(path_start_error_)
           << ",\"path_goal_error\":" << finiteOrMinusOne(path_goal_error_)
           << ",\"trajectory_samples\":" << trajectory_.size()
           << ",\"actual_travel_distance\":" << travel_distance
           << ",\"travel_efficiency\":" << efficiency
           << ",\"mean_tracking_error\":" << tracking.mean
           << ",\"rmse_tracking_error\":" << tracking.rmse
           << ",\"max_tracking_error\":" << tracking.maximum
           << ",\"goal_position_error\":" << finiteOrMinusOne(goal_position_error)
           << ",\"goal_yaw_error_deg\":" << finiteOrMinusOne(goal_yaw_error_deg)
           << ",\"replan_count\":" << replan_count_
           << ",\"global_path_messages\":" << global_path_messages_
           << ",\"local_path_messages\":" << local_path_messages_ << '}';
    return stream.str();
  }

  static double finiteOrMinusOne(double value) noexcept
  {
    return std::isfinite(value) ? value : -1.0;
  }

  void printSummary(
    bool passed, const ErrorStatistics & tracking, double travel_distance,
    double goal_position_error, double goal_yaw_error_deg) const
  {
    const double efficiency = initial_path_length_ > 0.0 ?
      travel_distance / initial_path_length_ : 0.0;
    std::ostringstream summary;
    summary << std::boolalpha << std::fixed << std::setprecision(3)
            << "\n==================================================\n"
            << "Nav2 Path Validation Result\n"
            << "==================================================\n\n"
            << "Navigation\n"
            << "  [" << (navigation_accepted_ ? "PASS" : "FAIL") <<
      "] NavigateToPose goal accepted\n"
            << "  [" << (navigation_succeeded_ ? "PASS" : "FAIL") <<
      "] NavigateToPose succeeded\n\n"
            << "Global Path\n"
            << "  [" << (path_received_ ? "PASS" : "FAIL") << "] Path received\n"
            << "  [" << (path_sufficient_ ? "PASS" : "FAIL") <<
      "] Path contains sufficient poses\n"
            << "  [" << (path_frame_valid_ ? "PASS" : "FAIL") <<
      "] Path frame valid\n"
            << "  [" << (std::isfinite(path_goal_error_) &&
    path_goal_error_ <= path_goal_tolerance_ ? "PASS" : "FAIL") <<
      "] Path ends near requested goal\n"
            << "  Frame             : " << validation_frame_ << '\n'
            << "  Poses             : " << initial_path_.size() << '\n'
            << "  Initial length    : " << initial_path_length_ << " m\n"
            << "  Latest length     : " << latest_path_length_ << " m\n"
            << "  Start error       : " << finiteOrMinusOne(path_start_error_) << " m\n"
            << "  Path-goal error   : " << finiteOrMinusOne(path_goal_error_) << " m\n"
            << "  Replan count      : " << replan_count_ << '\n'
            << "  Local path seen   : " << local_path_received_ << "\n\n"
            << "Robot Trajectory\n"
            << "  [" << (trajectory_.size() > 1U ? "PASS" : "FAIL") <<
      "] Robot trajectory collected\n"
            << "  Samples           : " << trajectory_.size() << '\n'
            << "  Travel distance   : " << travel_distance << " m\n"
            << "  Travel efficiency : " << efficiency << "\n\n"
            << "Path Tracking\n"
            << "  [" << (tracking.valid && tracking.rmse <= max_tracking_rmse_ ?
    "PASS" : "FAIL") << "] Tracking RMSE below threshold\n"
            << "  [" << (tracking.valid && tracking.maximum <= max_tracking_error_ ?
    "PASS" : "FAIL") << "] Maximum tracking error below threshold\n"
            << "  Mean error        : " << tracking.mean << " m\n"
            << "  RMSE              : " << tracking.rmse << " m\n"
            << "  Maximum error     : " << tracking.maximum << " m\n\n"
            << "Goal Accuracy\n"
            << "  [" << (goal_position_error <= goal_position_tolerance_ ? "PASS" : "FAIL") <<
      "] Final position error below threshold\n"
            << "  [" << (goal_yaw_error_deg <= goal_yaw_tolerance_deg_ ? "PASS" : "FAIL") <<
      "] Final yaw error below threshold\n"
            << "  Position error    : " << finiteOrMinusOne(goal_position_error) << " m\n"
            << "  Yaw error         : " << finiteOrMinusOne(goal_yaw_error_deg) << " deg\n\n"
            << "--------------------------------------------------\n"
            << "Result: " << (passed ? "PASS" : "FAIL") << '\n'
            << "--------------------------------------------------";
    if (passed) {
      RCLCPP_INFO(get_logger(), "%s", summary.str().c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "%s", summary.str().c_str());
    }
  }

  void writeResults(
    bool passed, const ErrorStatistics & tracking, double travel_distance,
    double goal_position_error, double goal_yaw_error_deg) const
  {
    try {
      std::filesystem::create_directories(results_directory_);
      const std::string stem = "nav2_path_validation_" + timestampForFilename();
      const auto summary_path = std::filesystem::path(results_directory_) / (stem + ".csv");
      const auto planned_path = std::filesystem::path(results_directory_) /
        (stem + "_planned_path.csv");
      const auto trajectory_path =
        std::filesystem::path(results_directory_) / (stem + "_actual_trajectory.csv");

      std::ofstream summary(summary_path);
      summary << "timestamp,scenario,goal_x,goal_y,global_path_length,latest_path_length,"
        "actual_travel_distance,travel_efficiency,mean_tracking_error,"
        "rmse_tracking_error,max_tracking_error,goal_position_error,"
        "goal_yaw_error_deg,replan_count,result\n";
      summary << timestampForFilename() << ',' << scenario_ << ',' << goal_.pose.position.x << ','
              << goal_.pose.position.y << ',' << initial_path_length_ << ',' <<
        latest_path_length_ << ','
              << travel_distance << ','
              << (initial_path_length_ > 0.0 ? travel_distance / initial_path_length_ : 0.0) << ','
              << tracking.mean << ',' << tracking.rmse << ',' << tracking.maximum << ','
              << finiteOrMinusOne(goal_position_error) << ',' <<
        finiteOrMinusOne(goal_yaw_error_deg)
              << ',' << replan_count_ << ',' << (passed ? "PASS" : "FAIL") << '\n';

      std::ofstream planned(planned_path);
      planned << "index,x,y\n";
      for (std::size_t index = 0U; index < current_path_.size(); ++index) {
        planned << index << ',' << current_path_[index].x << ',' << current_path_[index].y << '\n';
      }

      std::ofstream actual(trajectory_path);
      actual << "index,stamp_seconds,x,y,yaw,tracking_error\n";
      for (std::size_t index = 0U; index < trajectory_.size(); ++index) {
        const auto & sample = trajectory_[index];
        actual << index << ',' << sample.stamp_seconds << ',' << sample.point.x << ','
               << sample.point.y << ',' << sample.yaw << ','
               << finiteOrMinusOne(sample.tracking_error) << '\n';
      }
      RCLCPP_INFO(get_logger(), "Validation CSV files written under %s",
          results_directory_.c_str());
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Failed to write validation CSV: %s", exception.what());
    }
  }

  std::string global_path_topic_;
  std::string local_path_topic_;
  std::string odom_topic_;
  std::string goal_topic_;
  std::string mission_status_topic_;
  std::string action_status_topic_;
  std::string validation_frame_;
  std::string robot_base_frame_;
  std::string pose_source_;
  std::string completion_source_;
  std::string scenario_;
  std::string results_directory_;
  double trajectory_sample_distance_{0.02};
  double sample_period_seconds_{0.05};
  double path_goal_tolerance_{0.25};
  double goal_position_tolerance_{0.25};
  double goal_yaw_tolerance_deg_{15.0};
  double max_tracking_rmse_{0.30};
  double max_tracking_error_{0.70};
  double replan_change_threshold_{0.20};
  bool write_csv_{true};

  ValidationState state_{ValidationState::WAITING_FOR_NAVIGATION};
  bool active_session_{false};
  bool finalized_{false};
  bool have_goal_{false};
  bool navigation_accepted_{false};
  bool navigation_succeeded_{false};
  bool path_received_{false};
  bool path_sufficient_{false};
  bool path_frame_valid_{false};
  bool local_path_received_{false};
  geometry_msgs::msg::PoseStamped goal_;
  rclcpp::Time session_started_{0, 0, RCL_ROS_TIME};
  std::vector<Point2D> initial_path_;
  std::vector<Point2D> current_path_;
  std::vector<PoseSample> trajectory_;
  std::vector<double> tracking_errors_;
  double initial_path_length_{0.0};
  double latest_path_length_{0.0};
  double path_start_error_{std::numeric_limits<double>::quiet_NaN()};
  double path_goal_error_{std::numeric_limits<double>::quiet_NaN()};
  std::size_t replan_count_{0U};
  std::size_t global_path_messages_{0U};
  std::size_t local_path_messages_{0U};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<arm_navi_safety_interfaces::msg::MissionStatus>::SharedPtr
    mission_status_subscription_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr action_status_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finalize_service_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
};

std::shared_ptr<rclcpp::Node> makePathValidatorNode(const rclcpp::NodeOptions & options)
{
  return std::make_shared<PathValidatorNode>(options);
}

}  // namespace nav2_path_validator

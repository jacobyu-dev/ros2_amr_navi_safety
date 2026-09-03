#include "mission_manager/mission_manager_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mission_manager
{
namespace
{

rclcpp::QoS reliableQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

std::int64_t steadyNowNs() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void validateAbsoluteName(const std::string & value, const std::string & parameter_name)
{
  if (value.empty() || value.front() != '/') {
    throw std::invalid_argument(parameter_name + " must be an absolute ROS name");
  }
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(orientation);
}

std::uint8_t missionStateToMessage(MissionState state) noexcept
{
  using Message = arm_navi_safety_interfaces::msg::MissionStatus;
  switch (state) {
    case MissionState::IDLE: return Message::IDLE;
    case MissionState::RUNNING: return Message::RUNNING;
    case MissionState::PAUSED: return Message::PAUSED;
    case MissionState::COMPLETED: return Message::COMPLETED;
    case MissionState::FAILED: return Message::FAILED;
    case MissionState::CANCELED: return Message::CANCELED;
  }
  return Message::IDLE;
}

std::uint8_t navigationStateToMessage(NavigationState state) noexcept
{
  using Message = arm_navi_safety_interfaces::msg::MissionStatus;
  switch (state) {
    case NavigationState::IDLE: return Message::NAVIGATION_IDLE;
    case NavigationState::WAITING_FOR_GOAL: return Message::WAITING_FOR_GOAL;
    case NavigationState::NAVIGATING: return Message::NAVIGATING;
    case NavigationState::PAUSED: return Message::NAVIGATION_PAUSED;
    case NavigationState::ARRIVED: return Message::ARRIVED;
    case NavigationState::FAILED: return Message::NAVIGATION_FAILED;
    case NavigationState::CANCELED: return Message::NAVIGATION_CANCELED;
  }
  return Message::NAVIGATION_IDLE;
}

}  // namespace

MissionManagerNode::MissionManagerNode(const rclcpp::NodeOptions & options)
: Node("mission_manager_node", options)
{
  const auto safety_topic = this->declare_parameter<std::string>("safety_topic", "/safety/status");
  const auto status_topic = this->declare_parameter<std::string>("mission_status_topic", "/mission/status");
  goal_pose_topic_ = this->declare_parameter<std::string>("goal_pose_topic", "/mission/goal_pose");
  const auto navigate_action_name = this->declare_parameter<std::string>(
    "navigate_action_name", "/navigate_to_pose");
  const auto start_service_name = this->declare_parameter<std::string>(
    "start_service", "/mission/start");
  const auto cancel_service_name = this->declare_parameter<std::string>(
    "cancel_service", "/mission/cancel");
  const auto pause_service_name = this->declare_parameter<std::string>(
    "pause_service", "/mission/pause");
  const auto resume_service_name = this->declare_parameter<std::string>(
    "resume_service", "/mission/resume");
  const auto configured_executor_threads = this->declare_parameter<int>("executor_threads", 4);
  require_safety_clear_ = this->declare_parameter<bool>("require_safety_clear", true);
  goal_frame_ = this->declare_parameter<std::string>("goal_frame", "map");
  const auto mission_id = this->declare_parameter<std::int64_t>("mission_id", 1);
  mission_goal_xs_ = this->declare_parameter<std::vector<double>>(
    "mission_goal_xs", {1.0, 2.0});
  mission_goal_ys_ = this->declare_parameter<std::vector<double>>(
    "mission_goal_ys", {0.0, 1.0});
  mission_goal_yaws_ = this->declare_parameter<std::vector<double>>(
    "mission_goal_yaws", {0.0, 1.57});

  validateAbsoluteName(safety_topic, "safety_topic");
  validateAbsoluteName(status_topic, "mission_status_topic");
  validateAbsoluteName(goal_pose_topic_, "goal_pose_topic");
  validateAbsoluteName(navigate_action_name, "navigate_action_name");
  validateAbsoluteName(start_service_name, "start_service");
  validateAbsoluteName(cancel_service_name, "cancel_service");
  validateAbsoluteName(pause_service_name, "pause_service");
  validateAbsoluteName(resume_service_name, "resume_service");
  if (configured_executor_threads <= 0) {
    throw std::invalid_argument("executor_threads must be positive");
  }
  if (mission_id <= 0) {
    throw std::invalid_argument("mission_id must be positive");
  }
  if (goal_frame_.empty()) {
    throw std::invalid_argument("goal_frame must not be empty");
  }
  if (mission_goal_xs_.empty() || mission_goal_xs_.size() != mission_goal_ys_.size() ||
    mission_goal_xs_.size() != mission_goal_yaws_.size())
  {
    throw std::invalid_argument(
            "mission_goal_xs, mission_goal_ys, and mission_goal_yaws must be non-empty and equal-sized");
  }
  configured_mission_id_ = static_cast<std::uint64_t>(mission_id);
  executor_threads_ = static_cast<std::size_t>(configured_executor_threads);

  // Inputs may arrive together under a MultiThreadedExecutor.  The state mutex
  // protects the compound lifecycle state; service callbacks are serial only to
  // keep independent user requests ordered.
  input_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  service_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions input_options;
  input_options.callback_group = input_callback_group_;
  safety_subscription_ = this->create_subscription<arm_navi_safety_interfaces::msg::SafetyStatus>(
    safety_topic, reliableQos(),
    std::bind(&MissionManagerNode::handleSafetyState, this, std::placeholders::_1), input_options);
  goal_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_pose_topic_, reliableQos(),
    std::bind(&MissionManagerNode::handleGoalPose, this, std::placeholders::_1), input_options);
  status_publisher_ = this->create_publisher<arm_navi_safety_interfaces::msg::MissionStatus>(
    status_topic, reliableQos());
  start_service_ = this->create_service<Trigger>(
    start_service_name, std::bind(&MissionManagerNode::handleMissionStart, this,
    std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), service_callback_group_);
  cancel_service_ = this->create_service<Trigger>(
    cancel_service_name, std::bind(&MissionManagerNode::handleMissionCancel, this,
    std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), service_callback_group_);
  pause_service_ = this->create_service<Trigger>(
    pause_service_name, std::bind(&MissionManagerNode::handleMissionPause, this,
    std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), service_callback_group_);
  resume_service_ = this->create_service<Trigger>(
    resume_service_name, std::bind(&MissionManagerNode::handleMissionResume, this,
    std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), service_callback_group_);
  navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, navigate_action_name);

  RCLCPP_INFO(
    this->get_logger(), "Mission manager started: action=%s, safety=%s, explicit resume policy=%s",
    navigate_action_name.c_str(), safety_topic.c_str(), require_safety_clear_ ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(), "External navigation goals: %s", goal_pose_topic_.c_str());
  publishStatus();
}

std::size_t MissionManagerNode::executorThreads() const noexcept {return executor_threads_;}

Mission MissionManagerNode::configuredMission() const
{
  Mission mission;
  mission.mission_id = configured_mission_id_;
  mission.goals.reserve(mission_goal_xs_.size());
  for (std::size_t index = 0U; index < mission_goal_xs_.size(); ++index) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = goal_frame_;
    goal.pose.position.x = mission_goal_xs_.at(index);
    goal.pose.position.y = mission_goal_ys_.at(index);
    goal.pose.orientation = yawToQuaternion(mission_goal_yaws_.at(index));
    mission.goals.push_back(std::move(goal));
  }
  return mission;
}

MissionManagerNode::Snapshot MissionManagerNode::snapshotLocked() const
{
  return Snapshot{
    state_machine_.missionState(), state_machine_.navigationState(), current_mission_.mission_id,
    current_mission_.current_goal_index, current_mission_.goals.size(), remaining_distance_,
    navigation_time_sec_, number_of_recoveries_, reason_};
}

void MissionManagerNode::setReasonLocked(std::string reason)
{
  reason_ = std::move(reason);
}

void MissionManagerNode::logTransition(const Snapshot & before, const Snapshot & after) const
{
  if (before.mission_state != after.mission_state) {
    RCLCPP_INFO(this->get_logger(), "Mission state: %s -> %s", toString(before.mission_state),
      toString(after.mission_state));
  }
  if (before.navigation_state != after.navigation_state) {
    RCLCPP_INFO(this->get_logger(), "Navigation state: %s -> %s", toString(before.navigation_state),
      toString(after.navigation_state));
  }
}

void MissionManagerNode::publishStatus()
{
  Snapshot state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = snapshotLocked();
  }
  arm_navi_safety_interfaces::msg::MissionStatus message;
  message.header.stamp = this->now();
  message.mission_id = state.mission_id;
  message.mission_state = missionStateToMessage(state.mission_state);
  message.navigation_state = navigationStateToMessage(state.navigation_state);
  message.current_goal_index = static_cast<std::uint32_t>(state.current_goal_index);
  message.total_goals = static_cast<std::uint32_t>(state.total_goals);
  message.distance_remaining = state.distance_remaining;
  message.navigation_time_sec = state.navigation_time_sec;
  message.number_of_recoveries = state.number_of_recoveries;
  message.reason = state.reason;
  status_publisher_->publish(message);
}

bool MissionManagerNode::isSafetyStop(std::uint8_t level) const noexcept
{
  using SafetyStatus = arm_navi_safety_interfaces::msg::SafetyStatus;
  return level == SafetyStatus::STOP || level == SafetyStatus::ERROR || level == SafetyStatus::UNKNOWN;
}

bool MissionManagerNode::isSafetyMotionPermitted(std::uint8_t level) const noexcept
{
  using SafetyStatus = arm_navi_safety_interfaces::msg::SafetyStatus;
  return level == SafetyStatus::SAFE || level == SafetyStatus::WARNING;
}

void MissionManagerNode::handleMissionStart(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  if (!navigation_client_->action_server_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "NavigateToPose action server is unavailable");
    response->success = false;
    response->message = "NavigateToPose action server is not ready";
    return;
  }

  const auto mission = configuredMission();
  Snapshot before;
  Snapshot after;
  bool started = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    before = snapshotLocked();
    if (require_safety_clear_ && !safety_motion_permitted_) {
      response->message = "mission start rejected: safety is not SAFE/WARNING";
    } else if (!state_machine_.start(mission)) {
      response->message = "mission start rejected from state " + std::string(toString(before.mission_state));
    } else {
      current_mission_ = mission;
      cancel_reason_ = CancelReason::NONE;
      setReasonLocked("mission started");
      response->message = "mission " + std::to_string(current_mission_.mission_id) + " started";
      started = true;
    }
    after = snapshotLocked();
  }
  response->success = started;
  logTransition(before, after);
  publishStatus();
  if (started) {
    RCLCPP_INFO(this->get_logger(), "Mission %lu started with %zu waypoints", configured_mission_id_,
      mission.goals.size());
    sendCurrentGoal();
  }
}

void MissionManagerNode::handleGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  if (message->header.frame_id.empty()) {
    RCLCPP_WARN(this->get_logger(), "Navigation goal rejected: frame_id is empty");
    return;
  }
  if (!navigation_client_->action_server_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "NavigateToPose action server unavailable; external goal ignored");
    return;
  }

  Mission mission;
  mission.mission_id = configured_mission_id_;
  mission.goals.push_back(*message);
  Snapshot before;
  Snapshot after;
  bool started = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    before = snapshotLocked();
    if (require_safety_clear_ && !safety_motion_permitted_) {
      RCLCPP_WARN(this->get_logger(), "Navigation goal rejected: safety does not permit motion");
    } else if (!state_machine_.start(mission)) {
      RCLCPP_WARN(
        this->get_logger(), "Navigation goal rejected: mission state is %s",
        toString(before.mission_state));
    } else {
      current_mission_ = std::move(mission);
      cancel_reason_ = CancelReason::NONE;
      setReasonLocked("external navigation goal received");
      started = true;
    }
    after = snapshotLocked();
  }
  if (!started) {
    return;
  }
  RCLCPP_INFO(
    this->get_logger(), "Navigation goal received: frame=%s x=%.3f y=%.3f",
    message->header.frame_id.c_str(), message->pose.position.x, message->pose.position.y);
  logTransition(before, after);
  publishStatus();
  sendCurrentGoal();
}

void MissionManagerNode::cancelGoal(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
  if (goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Canceling active navigation goal");
    static_cast<void>(navigation_client_->async_cancel_goal(goal_handle));
  }
}

void MissionManagerNode::handleMissionCancel(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  Snapshot before;
  Snapshot after;
  GoalHandleNavigateToPose::SharedPtr goal_to_cancel;
  bool canceled = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    before = snapshotLocked();
    canceled = state_machine_.cancel();
    if (canceled) {
      cancel_reason_ = CancelReason::MISSION_CANCEL;
      goal_to_cancel = active_goal_handle_;
      setReasonLocked("mission canceled by request");
      response->message = "mission cancel accepted";
    } else {
      response->message = "mission cancel rejected from state " + std::string(toString(before.mission_state));
    }
    after = snapshotLocked();
  }
  response->success = canceled;
  logTransition(before, after);
  publishStatus();
  cancelGoal(goal_to_cancel);
}

void MissionManagerNode::handleMissionPause(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  // A manual pause uses the identical conservative policy as a safety stop.
  Snapshot before;
  Snapshot after;
  GoalHandleNavigateToPose::SharedPtr goal_to_cancel;
  bool paused = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    before = snapshotLocked();
    paused = state_machine_.pauseBySafety();
    if (paused) {
      cancel_reason_ = CancelReason::MANUAL_PAUSE;
      goal_to_cancel = active_goal_handle_;
      setReasonLocked("mission paused by request");
      response->message = "mission paused";
    } else {
      response->message = "mission pause rejected from state " + std::string(toString(before.mission_state));
    }
    after = snapshotLocked();
  }
  response->success = paused;
  logTransition(before, after);
  publishStatus();
  cancelGoal(goal_to_cancel);
}

void MissionManagerNode::handleMissionResume(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  Snapshot before;
  Snapshot after;
  bool resumed = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    before = snapshotLocked();
    resumed = state_machine_.resume(!require_safety_clear_ || safety_motion_permitted_);
    if (resumed) {
      cancel_reason_ = CancelReason::NONE;
      setReasonLocked("mission resumed by explicit request");
      response->message = "mission resumed";
    } else {
      response->message = "mission resume rejected: state must be PAUSED and safety must permit motion";
    }
    after = snapshotLocked();
  }
  response->success = resumed;
  logTransition(before, after);
  publishStatus();
  if (resumed) {
    RCLCPP_INFO(this->get_logger(), "Mission resumed at waypoint %zu", after.current_goal_index + 1U);
    sendCurrentGoal();
  }
}

void MissionManagerNode::handleSafetyState(
  const arm_navi_safety_interfaces::msg::SafetyStatus::SharedPtr message)
{
  Snapshot before;
  Snapshot after;
  GoalHandleNavigateToPose::SharedPtr goal_to_cancel;
  bool transitioned = false;
  bool safety_recovered = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    before = snapshotLocked();
    const bool was_motion_permitted = safety_motion_permitted_;
    safety_motion_permitted_ = isSafetyMotionPermitted(message->level) && message->data_valid;
    safety_recovered = !was_motion_permitted && safety_motion_permitted_;
    if (isSafetyStop(message->level) || !message->data_valid) {
      transitioned = state_machine_.pauseBySafety();
      if (transitioned) {
        cancel_reason_ = CancelReason::SAFETY_STOP;
        goal_to_cancel = active_goal_handle_;
        setReasonLocked("paused by safety: " + message->reason);
      }
    }
    after = snapshotLocked();
  }
  if (transitioned) {
    RCLCPP_WARN(this->get_logger(), "Safety stop received: %s", message->reason.c_str());
    const auto mission_stop_steady_ns = steadyNowNs();
    if (message->decision_time_steady_ns > 0 &&
      mission_stop_steady_ns >= message->decision_time_steady_ns)
    {
      const auto reaction_ms = static_cast<double>(
        mission_stop_steady_ns - message->decision_time_steady_ns) / 1.0e6;
      if (message->fault_time_steady_ns > 0 &&
        mission_stop_steady_ns >= message->fault_time_steady_ns)
      {
        const auto end_to_end_ms = static_cast<double>(
          mission_stop_steady_ns - message->fault_time_steady_ns) / 1.0e6;
        RCLCPP_INFO(
          this->get_logger(),
          "[SafetyLatency][event=%llu][%s] Safety-to-mission: %.3f ms End-to-end: %.3f ms",
          static_cast<unsigned long long>(message->latency_event_id),
          message->latency_fault_type.c_str(), reaction_ms, end_to_end_ms);
      } else {
        RCLCPP_INFO(
          this->get_logger(), "[SafetyLatency] Safety-to-mission: %.3f ms", reaction_ms);
      }
    }
    logTransition(before, after);
    publishStatus();
    cancelGoal(goal_to_cancel);
  } else if (isSafetyMotionPermitted(message->level) && message->data_valid) {
    if (safety_recovered) {
      RCLCPP_INFO(this->get_logger(), "Safety recovered/permitted; explicit /mission/resume is required");
    }
    // Also make the current state observable for subscribers that connect
    // after startup. Safety recovery intentionally changes no lifecycle state.
    publishStatus();
  }
}

void MissionManagerNode::sendCurrentGoal()
{
  NavigateToPose::Goal goal;
  std::uint64_t sequence = 0U;
  Snapshot before;
  Snapshot after;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    before = snapshotLocked();
    if (!state_machine_.beginNavigation() || !current_mission_.hasCurrentGoal()) {
      return;
    }
    goal.pose = current_mission_.currentGoal();
    goal.pose.header.stamp = this->now();
    active_goal_handle_.reset();
    active_goal_sequence_ = next_goal_sequence_++;
    sequence = active_goal_sequence_;
    remaining_distance_ = -1.0F;
    navigation_time_sec_ = 0.0F;
    number_of_recoveries_ = 0;
    navigation_started_steady_ns_ = steadyNowNs();
    setReasonLocked("sending waypoint " + std::to_string(current_mission_.current_goal_index + 1U) + "/" +
      std::to_string(current_mission_.goals.size()));
    after = snapshotLocked();
  }
  logTransition(before, after);
  publishStatus();
  RCLCPP_INFO(this->get_logger(), "Sending NavigateToPose waypoint %zu/%zu", after.current_goal_index + 1U,
    after.total_goals);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback = [this, sequence](GoalHandleNavigateToPose::SharedPtr handle) {
      handleNavigationAccepted(sequence, std::move(handle));
    };
  options.feedback_callback = [this, sequence](GoalHandleNavigateToPose::SharedPtr handle,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      handleNavigationFeedback(sequence, std::move(handle), feedback);
    };
  options.result_callback = [this, sequence](const GoalHandleNavigateToPose::WrappedResult & result) {
      handleNavigationResult(sequence, result);
    };
  try {
    static_cast<void>(navigation_client_->async_send_goal(goal, options));
  } catch (const std::exception & exception) {
    Snapshot failure_before;
    Snapshot failure_after;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      failure_before = snapshotLocked();
      if (sequence == active_goal_sequence_ && state_machine_.navigationFailed()) {
        setReasonLocked("failed to send navigation goal: " + std::string(exception.what()));
      }
      failure_after = snapshotLocked();
    }
    logTransition(failure_before, failure_after);
    publishStatus();
  }
}

void MissionManagerNode::handleNavigationAccepted(
  std::uint64_t sequence, GoalHandleNavigateToPose::SharedPtr goal_handle)
{
  Snapshot before;
  Snapshot after;
  GoalHandleNavigateToPose::SharedPtr goal_to_cancel;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    before = snapshotLocked();
    if (sequence != active_goal_sequence_) {
      return;
    }
    if (!goal_handle) {
      if (state_machine_.navigationFailed()) {
        setReasonLocked("navigation goal rejected by action server");
      }
    } else {
      active_goal_handle_ = goal_handle;
      if (state_machine_.missionState() == MissionState::PAUSED ||
        state_machine_.missionState() == MissionState::CANCELED)
      {
        goal_to_cancel = goal_handle;
      } else {
        setReasonLocked("navigation goal accepted");
      }
    }
    after = snapshotLocked();
  }
  logTransition(before, after);
  publishStatus();
  if (goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Navigation goal accepted");
  }
  cancelGoal(goal_to_cancel);
}

void MissionManagerNode::handleNavigationFeedback(
  std::uint64_t sequence, GoalHandleNavigateToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (sequence != active_goal_sequence_ ||
      state_machine_.navigationState() != NavigationState::NAVIGATING)
    {
      return;
    }
    remaining_distance_ = feedback->distance_remaining;
    number_of_recoveries_ = feedback->number_of_recoveries;
    if (navigation_started_steady_ns_ > 0) {
      navigation_time_sec_ = static_cast<float>(
        static_cast<double>(steadyNowNs() - navigation_started_steady_ns_) / 1.0e9);
    }
  }
  // Feedback can be high-frequency; make it observable on /mission/status
  // without emitting a log line for every Nav2 update.
  publishStatus();
}

void MissionManagerNode::handleNavigationResult(
  std::uint64_t sequence, const GoalHandleNavigateToPose::WrappedResult & result)
{
  Snapshot before;
  Snapshot after;
  bool send_next = false;
  bool handled = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    before = snapshotLocked();
    if (sequence != active_goal_sequence_) {
      return;
    }
    active_goal_handle_.reset();
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        handled = state_machine_.goalSucceeded(current_mission_);
        if (handled) {
          setReasonLocked(state_machine_.missionState() == MissionState::COMPLETED ?
            "all waypoints reached" : "waypoint reached");
          send_next = state_machine_.navigationState() == NavigationState::WAITING_FOR_GOAL;
        }
        break;
      case rclcpp_action::ResultCode::ABORTED:
        handled = state_machine_.navigationFailed();
        if (handled) {
          setReasonLocked("navigation action aborted");
        }
        break;
      case rclcpp_action::ResultCode::CANCELED:
        // Safety and user transitions already moved state before cancellation.
        handled = true;
        if (cancel_reason_ == CancelReason::NONE && state_machine_.missionState() == MissionState::RUNNING) {
          static_cast<void>(state_machine_.cancel());
          setReasonLocked("navigation action canceled externally");
        }
        break;
      default:
        handled = state_machine_.navigationFailed();
        if (handled) {
          setReasonLocked("navigation action returned unknown result");
        }
        break;
    }
    after = snapshotLocked();
  }
  if (!handled) {
    return;
  }
  logTransition(before, after);
  publishStatus();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "Waypoint reached");
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_ERROR(this->get_logger(), "Navigation failed: action aborted");
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_INFO(this->get_logger(), "Navigation canceled");
  }
  if (send_next) {
    sendCurrentGoal();
  } else if (after.mission_state == MissionState::COMPLETED) {
    RCLCPP_INFO(this->get_logger(), "Mission %lu completed", after.mission_id);
  }
}

}  // namespace mission_manager

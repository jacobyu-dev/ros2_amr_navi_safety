#include "mission_manager/fake_navigate_to_pose_server.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <thread>
#include <utility>

namespace mission_manager
{
namespace
{

bool isValidOutcome(const std::string & outcome)
{
  return outcome == "success" || outcome == "abort" || outcome == "hold";
}

}  // namespace

FakeNavigateToPoseServer::FakeNavigateToPoseServer(const rclcpp::NodeOptions & options)
: Node("fake_navigate_to_pose_server", options)
{
  const auto action_name = this->declare_parameter<std::string>(
    "navigate_action_name", "/navigate_to_pose");
  const auto delay_ms = this->declare_parameter<int>("result_delay_ms", 100);
  const auto cancel_count_topic = this->declare_parameter<std::string>(
    "cancel_count_topic", "/fake_navigate_to_pose/cancel_count");
  result_sequence_ = this->declare_parameter<std::vector<std::string>>(
    "result_sequence", {"hold"});
  reject_goals_ = this->declare_parameter<bool>("reject_goals", false);
  if (action_name.empty() || action_name.front() != '/') {
    throw std::invalid_argument("navigate_action_name must be an absolute ROS name");
  }
  if (cancel_count_topic.empty() || cancel_count_topic.front() != '/') {
    throw std::invalid_argument("cancel_count_topic must be an absolute ROS name");
  }
  if (delay_ms < 0 || result_sequence_.empty()) {
    throw std::invalid_argument("result_delay_ms must be non-negative and result_sequence non-empty");
  }
  for (const auto & outcome : result_sequence_) {
    if (!isValidOutcome(outcome)) {
      throw std::invalid_argument("result_sequence supports only success, abort, or hold");
    }
  }
  result_delay_ = std::chrono::milliseconds{delay_ms};
  cancel_count_publisher_ = this->create_publisher<std_msgs::msg::UInt64>(cancel_count_topic, 10);
  parameter_callback_ = this->add_on_set_parameters_callback(
    std::bind(&FakeNavigateToPoseServer::updateParameters, this, std::placeholders::_1));
  server_ = rclcpp_action::create_server<Action>(
    this, action_name,
    std::bind(&FakeNavigateToPoseServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&FakeNavigateToPoseServer::handleCancel, this, std::placeholders::_1),
    std::bind(&FakeNavigateToPoseServer::handleAccepted, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Fake NavigateToPose server started at %s", action_name.c_str());
}

rcl_interfaces::msg::SetParametersResult FakeNavigateToPoseServer::updateParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "result_sequence") {
      const auto outcomes = parameter.as_string_array();
      if (outcomes.empty()) {
        result.successful = false;
        result.reason = "result_sequence must not be empty";
        return result;
      }
      for (const auto & outcome : outcomes) {
        if (!isValidOutcome(outcome)) {
          result.successful = false;
          result.reason = "result_sequence supports only success, abort, or hold";
          return result;
        }
      }
      std::lock_guard<std::mutex> lock(outcomes_mutex_);
      result_sequence_ = outcomes;
      accepted_goal_count_.store(0U);
    } else if (parameter.get_name() == "reject_goals") {
      std::lock_guard<std::mutex> lock(outcomes_mutex_);
      reject_goals_ = parameter.as_bool();
    }
  }
  return result;
}

rclcpp_action::GoalResponse FakeNavigateToPoseServer::handleGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>)
{
  std::lock_guard<std::mutex> lock(outcomes_mutex_);
  return reject_goals_ ? rclcpp_action::GoalResponse::REJECT :
         rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FakeNavigateToPoseServer::handleCancel(const std::shared_ptr<GoalHandle>)
{
  std_msgs::msg::UInt64 count;
  count.data = cancel_request_count_.fetch_add(1U) + 1U;
  cancel_count_publisher_->publish(count);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FakeNavigateToPoseServer::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal_index = accepted_goal_count_.fetch_add(1U);
  std::string outcome;
  {
    std::lock_guard<std::mutex> lock(outcomes_mutex_);
    outcome = result_sequence_.at(std::min(goal_index, result_sequence_.size() - 1U));
  }
  std::thread(&FakeNavigateToPoseServer::execute, this, goal_handle, std::move(outcome)).detach();
}

void FakeNavigateToPoseServer::execute(const std::shared_ptr<GoalHandle> goal_handle, std::string outcome)
{
  std::this_thread::sleep_for(result_delay_);
  auto feedback = std::make_shared<Action::Feedback>();
  feedback->current_pose = goal_handle->get_goal()->pose;
  feedback->distance_remaining = 0.0F;
  goal_handle->publish_feedback(feedback);
  if (outcome == "hold") {
    while (rclcpp::ok() && !goal_handle->is_canceling()) {
      std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(std::make_shared<Action::Result>());
    }
    return;
  }
  if (goal_handle->is_canceling()) {
    goal_handle->canceled(std::make_shared<Action::Result>());
  } else if (outcome == "success") {
    goal_handle->succeed(std::make_shared<Action::Result>());
  } else {
    goal_handle->abort(std::make_shared<Action::Result>());
  }
}

}  // namespace mission_manager

#ifndef MISSION_MANAGER__FAKE_NAVIGATE_TO_POSE_SERVER_HPP_
#define MISSION_MANAGER__FAKE_NAVIGATE_TO_POSE_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace mission_manager
{

/** Test-only deterministic implementation of Nav2's NavigateToPose action. */
class FakeNavigateToPoseServer : public rclcpp::Node
{
public:
  explicit FakeNavigateToPoseServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Action = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle, std::string outcome);
  rcl_interfaces::msg::SetParametersResult updateParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  std::mutex outcomes_mutex_;
  std::vector<std::string> result_sequence_;
  bool reject_goals_{false};
  std::atomic<std::size_t> accepted_goal_count_{0U};
  std::atomic<std::uint64_t> cancel_request_count_{0U};
  std::chrono::milliseconds result_delay_{100};
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr cancel_count_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

}  // namespace mission_manager

#endif  // MISSION_MANAGER__FAKE_NAVIGATE_TO_POSE_SERVER_HPP_

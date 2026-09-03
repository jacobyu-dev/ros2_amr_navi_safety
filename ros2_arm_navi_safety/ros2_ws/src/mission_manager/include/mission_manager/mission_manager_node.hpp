#ifndef MISSION_MANAGER__MISSION_MANAGER_NODE_HPP_
#define MISSION_MANAGER__MISSION_MANAGER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "arm_navi_safety_interfaces/msg/mission_status.hpp"
#include "arm_navi_safety_interfaces/msg/safety_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mission_manager/mission.hpp"
#include "mission_manager/mission_state_machine.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mission_manager
{

enum class CancelReason
{
  NONE,
  SAFETY_STOP,
  MANUAL_PAUSE,
  MISSION_CANCEL
};

class MissionManagerNode : public rclcpp::Node
{
public:
  explicit MissionManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  [[nodiscard]] std::size_t executorThreads() const noexcept;

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Trigger = std_srvs::srv::Trigger;

  struct Snapshot
  {
    MissionState mission_state{MissionState::IDLE};
    NavigationState navigation_state{NavigationState::IDLE};
    std::uint64_t mission_id{0U};
    std::size_t current_goal_index{0U};
    std::size_t total_goals{0U};
    float distance_remaining{-1.0F};
    float navigation_time_sec{0.0F};
    std::int16_t number_of_recoveries{0};
    std::string reason;
  };

  void handleMissionStart(
    const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);
  void handleMissionCancel(
    const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);
  void handleMissionPause(
    const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);
  void handleMissionResume(
    const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);
  void handleSafetyState(const arm_navi_safety_interfaces::msg::SafetyStatus::SharedPtr message);
  void handleGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr message);

  void sendCurrentGoal();
  void handleNavigationAccepted(
    std::uint64_t sequence, GoalHandleNavigateToPose::SharedPtr goal_handle);
  void handleNavigationFeedback(
    std::uint64_t sequence, GoalHandleNavigateToPose::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void handleNavigationResult(
    std::uint64_t sequence, const GoalHandleNavigateToPose::WrappedResult & result);
  void cancelGoal(const GoalHandleNavigateToPose::SharedPtr & goal_handle);

  [[nodiscard]] Mission configuredMission() const;
  [[nodiscard]] Snapshot snapshotLocked() const;
  void publishStatus();
  void logTransition(const Snapshot & before, const Snapshot & after) const;
  void setReasonLocked(std::string reason);
  [[nodiscard]] bool isSafetyStop(std::uint8_t level) const noexcept;
  [[nodiscard]] bool isSafetyMotionPermitted(std::uint8_t level) const noexcept;

  mutable std::mutex state_mutex_;
  MissionStateMachine state_machine_;
  Mission current_mission_;
  GoalHandleNavigateToPose::SharedPtr active_goal_handle_;
  std::uint64_t active_goal_sequence_{0U};
  std::uint64_t next_goal_sequence_{1U};
  CancelReason cancel_reason_{CancelReason::NONE};
  bool safety_motion_permitted_{false};
  float remaining_distance_{-1.0F};
  float navigation_time_sec_{0.0F};
  std::int16_t number_of_recoveries_{0};
  std::int64_t navigation_started_steady_ns_{0};
  std::string reason_{"waiting for mission"};

  std::size_t executor_threads_{4U};
  bool require_safety_clear_{true};
  std::string goal_frame_{"map"};
  std::string goal_pose_topic_{"/mission/goal_pose"};
  std::uint64_t configured_mission_id_{1U};
  std::vector<double> mission_goal_xs_;
  std::vector<double> mission_goal_ys_;
  std::vector<double> mission_goal_yaws_;

  rclcpp::CallbackGroup::SharedPtr input_callback_group_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Subscription<arm_navi_safety_interfaces::msg::SafetyStatus>::SharedPtr safety_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;
  rclcpp::Publisher<arm_navi_safety_interfaces::msg::MissionStatus>::SharedPtr status_publisher_;
  rclcpp::Service<Trigger>::SharedPtr start_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<Trigger>::SharedPtr pause_service_;
  rclcpp::Service<Trigger>::SharedPtr resume_service_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
};

}  // namespace mission_manager

#endif  // MISSION_MANAGER__MISSION_MANAGER_NODE_HPP_

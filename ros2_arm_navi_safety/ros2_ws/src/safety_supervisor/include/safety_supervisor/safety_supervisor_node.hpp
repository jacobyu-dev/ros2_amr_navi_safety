#ifndef SAFETY_SUPERVISOR__SAFETY_SUPERVISOR_NODE_HPP_
#define SAFETY_SUPERVISOR__SAFETY_SUPERVISOR_NODE_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "arm_navi_safety_interfaces/msg/safety_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "safety_supervisor/safety_latency_monitor.hpp"
#include "safety_supervisor/safety_state_machine.hpp"

namespace safety_supervisor
{

/**
 * @brief Complete latest report from one required safety source.
 *
 * The fields describe one logical observation and must never be read from the
 * shared store individually. SafetySourceStore copies and replaces this value
 * under one mutex so evaluators see a consistent observation.
 */
struct SafetySourceState
{
  SafetyLevel level{SafetyLevel::UNKNOWN};
  rclcpp::Time last_received{0, 0, RCL_ROS_TIME};
  rclcpp::Time message_stamp{0, 0, RCL_ROS_TIME};
  bool received{false};
  bool has_message_stamp{false};
  bool data_valid{false};
  std::string reason;
  std::uint64_t latency_event_id{0U};
  std::string latency_fault_type;
  std::int64_t fault_time_steady_ns{0};
  std::int64_t detection_time_steady_ns{0};
};

/**
 * @brief Mutex-protected owner of all source observations.
 *
 * Reentrant subscription callbacks publish complete SafetySourceState values
 * through update(). The supervisor timer obtains a short-lived copy through
 * snapshot(), then releases the mutex before evaluating or publishing.
 */
class SafetySourceStore
{
public:
  /// Initialize one default state for every configured source before callbacks start.
  void initialize(const std::vector<std::string> & source_names);

  /// Replace a source's complete observation while holding the store mutex.
  void update(const std::string & source_name, SafetySourceState state);

  /// Return a consistent copy of all observations without exposing shared storage.
  [[nodiscard]] std::unordered_map<std::string, SafetySourceState> snapshot() const;

private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, SafetySourceState> sources_;
};

/**
 * @brief Aggregates LiDAR and TF safety reports into one system safety state.
 *
 * Source subscriptions are reentrant and may run concurrently. The evaluation
 * timer is mutually exclusive, so only one state-machine transition is made
 * at a time.
 */
class SafetySupervisorNode : public rclcpp::Node
{
public:
  /// Declare parameters, create callback groups, and wire safety I/O endpoints.
  SafetySupervisorNode();

  /// Return the validated thread count used by this executable's MTE.
  [[nodiscard]] std::size_t executor_threads() const noexcept;

  /// Return the atomic count of completed source-status callbacks.
  [[nodiscard]] std::uint64_t source_callback_count() const noexcept;

  /// Return the atomic count of completed timer evaluations.
  [[nodiscard]] std::uint64_t evaluation_callback_count() const noexcept;

private:
  using SafetyStatus = arm_navi_safety_interfaces::msg::SafetyStatus;

  /// Convert one input message to a local state and atomically replace its stored source state.
  void status_callback(const std::string & expected_source, const SafetyStatus::SharedPtr message);

  /// Evaluate one source snapshot, update the state machine, and publish outside the store lock.
  void evaluate_and_publish();

  /// Apply startup, timeout, validity, and severity policy to a consistent snapshot.
  [[nodiscard]] SafetyEvaluation evaluate_sources(
    const std::unordered_map<std::string, SafetySourceState> & sources,
    const rclcpp::Time & now) const;

  /// Translate the public message-level enum to the state-machine enum.
  [[nodiscard]] SafetyLevel status_level_to_safety_level(std::uint8_t level) const noexcept;

  /// Translate the state-machine result to the public message-level enum.
  [[nodiscard]] std::uint8_t state_to_status_level(SystemSafetyState state) const noexcept;

  /// Return true when receipt time or a supplied message timestamp exceeded the timeout.
  [[nodiscard]] bool source_is_stale(
    const SafetySourceState & state, const rclcpp::Time & now) const;

  /// Publish the already-computed result; this intentionally runs without a store lock.
  void publish_system_status(
    SystemSafetyState state, const SafetyEvaluation & evaluation, const rclcpp::Time & now,
    const std::unordered_map<std::string, SafetySourceState> & sources,
    std::int64_t decision_steady_ns);

  /// Record one completed event after the serial state-machine transition.
  void complete_latency_samples(
    const std::unordered_map<std::string, SafetySourceState> & sources,
    SystemSafetyState state, std::int64_t decision_steady_ns);

  void write_latency_csv(const SafetyLatencySample & sample);

  double evaluation_rate_hz_{20.0};
  double source_timeout_sec_{0.5};
  double startup_timeout_sec_{5.0};
  std::size_t executor_threads_{4U};
  bool enable_latency_measurement_{false};
  bool enable_latency_csv_{false};
  rclcpp::Time started_at_{0, 0, RCL_ROS_TIME};
  std::vector<std::string> required_sources_;
  // The only shared mutable source data. Access it only through update() or snapshot().
  SafetySourceStore source_store_;
  // Reentrant inputs increase responsiveness; SafetySourceStore protects their shared data.
  rclcpp::CallbackGroup::SharedPtr input_callback_group_;
  // Serializes state-machine transitions without blocking source updates.
  rclcpp::CallbackGroup::SharedPtr evaluation_callback_group_;
  std::vector<rclcpp::Subscription<SafetyStatus>::SharedPtr> status_subscriptions_;
  rclcpp::Publisher<SafetyStatus>::SharedPtr system_status_publisher_;
  rclcpp::TimerBase::SharedPtr evaluation_timer_;
  // Independent diagnostic counters; atomics avoid a mutex in hot callbacks.
  std::atomic<std::uint64_t> source_callback_count_{0U};
  std::atomic<std::uint64_t> evaluation_callback_count_{0U};
  // The monitor is independently mutex-protected because source callbacks are reentrant.
  SafetyLatencyMonitor latency_monitor_;
  std::ofstream latency_csv_;
  // Accessed only from evaluation_callback_group_, which is mutually exclusive.
  SafetyStateMachine state_machine_;
};

}  // namespace safety_supervisor

#endif  // SAFETY_SUPERVISOR__SAFETY_SUPERVISOR_NODE_HPP_

#include "safety_supervisor/safety_supervisor_node.hpp"

#include <chrono>
#include <fstream>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace safety_supervisor
{
namespace
{

// Status updates are safety-critical and should not be dropped under normal traffic.
rclcpp::QoS safety_status_qos()
{
  // Safety state should not be silently dropped under normal graph conditions.
  return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

// A zero ROS stamp means the sender did not provide a source timestamp.
bool is_zero_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0U;
}

std::int64_t steady_now_ns() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

void SafetySourceStore::initialize(const std::vector<std::string> & source_names)
{
  // Initialization is also protected so the store has one consistent setup path.
  std::lock_guard<std::mutex> lock(mutex_);
  sources_.clear();
  for (const auto & source_name : source_names) {
    sources_.emplace(source_name, SafetySourceState{});
  }
}

void SafetySourceStore::update(const std::string & source_name, SafetySourceState state)
{
  // Keep the critical section to one map replacement; no ROS work is done while locked.
  std::lock_guard<std::mutex> lock(mutex_);
  sources_.at(source_name) = std::move(state);
}

std::unordered_map<std::string, SafetySourceState> SafetySourceStore::snapshot() const
{
  // Return a private copy so the caller can evaluate and publish without holding this mutex.
  std::lock_guard<std::mutex> lock(mutex_);
  return sources_;
}

SafetySupervisorNode::SafetySupervisorNode()
: Node("safety_supervisor_node")
{
  evaluation_rate_hz_ = this->declare_parameter<double>("evaluation_rate_hz", 20.0);
  source_timeout_sec_ = this->declare_parameter<double>("source_timeout_sec", 0.5);
  startup_timeout_sec_ = this->declare_parameter<double>("startup_timeout_sec", 5.0);
  const auto configured_executor_threads = this->declare_parameter<int>("executor_threads", 4);
  required_sources_ = this->declare_parameter<std::vector<std::string>>(
    "required_sources", {"lidar_safety", "tf_monitor", "sensor_watchdog"});
  const auto source_topics = this->declare_parameter<std::vector<std::string>>(
    "source_topics", {"/safety/lidar/status", "/safety/tf/status", "/safety/watchdog/status"});
  const auto system_status_topic = this->declare_parameter<std::string>(
    "system_status_topic", "/safety/status");
  enable_latency_measurement_ = this->declare_parameter<bool>("enable_latency_measurement", false);
  enable_latency_csv_ = this->declare_parameter<bool>("enable_latency_csv", false);
  const auto latency_output_path = this->declare_parameter<std::string>(
    "latency_output_path", "safety_latency_results.csv");

  if (evaluation_rate_hz_ <= 0.0 || source_timeout_sec_ <= 0.0 || startup_timeout_sec_ <= 0.0) {
    throw std::invalid_argument(
            "evaluation_rate_hz, source_timeout_sec, and startup_timeout_sec must be positive");
  }
  if (configured_executor_threads <= 0) {
    throw std::invalid_argument("executor_threads must be positive");
  }
  executor_threads_ = static_cast<std::size_t>(configured_executor_threads);
  if (required_sources_.empty() || required_sources_.size() != source_topics.size()) {
    throw std::invalid_argument("required_sources and source_topics must be non-empty and same-sized");
  }
  if (system_status_topic.empty() || system_status_topic.front() != '/') {
    throw std::invalid_argument("system_status_topic must be an absolute ROS topic");
  }
  if (enable_latency_csv_ && !enable_latency_measurement_) {
    throw std::invalid_argument("enable_latency_csv requires enable_latency_measurement");
  }
  if (enable_latency_csv_ && latency_output_path.empty()) {
    throw std::invalid_argument("latency_output_path must not be empty when CSV is enabled");
  }

  std::unordered_set<std::string> unique_sources;
  for (std::size_t index = 0U; index < required_sources_.size(); ++index) {
    const auto & source = required_sources_[index];
    const auto & topic = source_topics[index];
    if (source.empty() || !unique_sources.insert(source).second) {
      throw std::invalid_argument("required_sources must contain unique, non-empty names");
    }
    if (topic.empty() || topic.front() != '/') {
      throw std::invalid_argument("each source_topics entry must be an absolute ROS topic");
    }
  }
  source_store_.initialize(required_sources_);

  started_at_ = this->now();
  // Source callbacks are reentrant: each builds a complete local state and
  // atomically replaces it in source_store_. The timer group remains serial so
  // state-machine transitions cannot overlap.
  input_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  evaluation_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  system_status_publisher_ = this->create_publisher<SafetyStatus>(system_status_topic, safety_status_qos());
  if (enable_latency_csv_) {
    latency_csv_.open(latency_output_path, std::ios::out | std::ios::trunc);
    if (!latency_csv_.is_open()) {
      throw std::runtime_error("failed to open latency CSV output: " + latency_output_path);
    }
    latency_csv_ << "event_id,fault_type,detection_ms,dispatch_ms,decision_ms,total_ms,result_state\n";
  }
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = input_callback_group_;
  for (std::size_t index = 0U; index < required_sources_.size(); ++index) {
    const auto source = required_sources_[index];
    status_subscriptions_.push_back(this->create_subscription<SafetyStatus>(
        source_topics[index], safety_status_qos(),
        [this, source](SafetyStatus::SharedPtr message) {status_callback(source, message);},
        subscription_options));
  }

  const auto period = std::chrono::duration<double>(1.0 / evaluation_rate_hz_);
  evaluation_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SafetySupervisorNode::evaluate_and_publish, this), evaluation_callback_group_);

  RCLCPP_INFO(
    this->get_logger(), "Safety supervisor started: %zu required sources, %.1f Hz evaluation",
    required_sources_.size(), evaluation_rate_hz_);
}

void SafetySupervisorNode::status_callback(
  const std::string & expected_source, const SafetyStatus::SharedPtr message)
{
  // Build the full report locally first. Only the final store replacement is shared.
  SafetySourceState state;
  state.received = true;
  const auto supervisor_start_steady_ns = steady_now_ns();
  state.last_received = this->now();
  state.has_message_stamp = !is_zero_stamp(message->header.stamp);
  if (state.has_message_stamp) {
    state.message_stamp = rclcpp::Time(message->header.stamp, this->get_clock()->get_clock_type());
  }
  state.latency_event_id = message->latency_event_id;
  state.latency_fault_type = message->latency_fault_type;
  state.fault_time_steady_ns = message->fault_time_steady_ns;
  state.detection_time_steady_ns = message->detection_time_steady_ns;
  if (enable_latency_measurement_ && state.latency_event_id != 0U) {
    latency_monitor_.record_supervisor_start(
      state.latency_event_id, state.latency_fault_type, state.fault_time_steady_ns,
      state.detection_time_steady_ns, supervisor_start_steady_ns);
  }

  if (message->source != expected_source) {
    state.level = SafetyLevel::FAULT;
    state.data_valid = false;
    state.reason = "received status with unexpected source '" + message->source + "'";
    source_store_.update(expected_source, std::move(state));
    source_callback_count_.fetch_add(1U);
    return;
  }

  state.level = status_level_to_safety_level(message->level);
  state.data_valid = message->data_valid;
  state.reason = message->reason;
  source_store_.update(expected_source, std::move(state));
  source_callback_count_.fetch_add(1U);
}

void SafetySupervisorNode::evaluate_and_publish()
{
  // The callback group serializes state_machine_, while the snapshot isolates
  // this evaluation from concurrent source updates.
  evaluation_callback_count_.fetch_add(1U);
  const auto now = this->now();
  const auto sources = source_store_.snapshot();
  const auto evaluation = evaluate_sources(sources, now);
  const auto previous_state = state_machine_.current_state();
  const auto current_state = state_machine_.update(evaluation);
  const auto decision_steady_ns = steady_now_ns();
  if (enable_latency_measurement_) {
    complete_latency_samples(sources, current_state, decision_steady_ns);
  }
  if (current_state != previous_state) {
    if (current_state == SystemSafetyState::FAULT) {
      RCLCPP_ERROR(this->get_logger(), "Safety state transition: %s -> %s (%s)",
        to_string(previous_state), to_string(current_state), evaluation.reason.c_str());
    } else if (current_state == SystemSafetyState::WARNING || current_state == SystemSafetyState::STOP) {
      RCLCPP_WARN(this->get_logger(), "Safety state transition: %s -> %s (%s)",
        to_string(previous_state), to_string(current_state), evaluation.reason.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Safety state transition: %s -> %s (%s)",
        to_string(previous_state), to_string(current_state), evaluation.reason.c_str());
    }
  }
  publish_system_status(current_state, evaluation, now, sources, decision_steady_ns);
}

void SafetySupervisorNode::complete_latency_samples(
  const std::unordered_map<std::string, SafetySourceState> & sources, SystemSafetyState state,
  std::int64_t decision_steady_ns)
{
  // This is called from the mutually exclusive evaluation group immediately after
  // state_machine_.update(), so its timestamp is T3 for the decision just made.
  for (const auto & item : sources) {
    const auto & source = item.second;
    if (source.latency_event_id == 0U) {
      continue;
    }
    const auto sample = latency_monitor_.complete_decision(
      source.latency_event_id, decision_steady_ns, to_string(state));
    if (!sample.has_value()) {
      continue;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "[Latency][event=%llu][%s] Detection: %.3f ms Dispatch: %.3f ms Decision: %.3f ms "
      "Total: %.3f ms State: %s",
      static_cast<unsigned long long>(sample->event_id), sample->fault_type.c_str(),
      sample->detection_latency_ms(), sample->dispatch_latency_ms(),
      sample->decision_latency_ms(), sample->total_latency_ms(), sample->result_state.c_str());
    const auto summary = latency_monitor_.summary(sample->fault_type);
    RCLCPP_INFO(
      this->get_logger(),
      "[Latency Summary][%s] samples=%zu total mean=%.3f ms median=%.3f ms p95=%.3f ms "
      "p99=%.3f ms max=%.3f ms",
      sample->fault_type.c_str(), summary.total.count, summary.total.mean_ms, summary.total.median_ms,
      summary.total.p95_ms, summary.total.p99_ms, summary.total.max_ms);
    write_latency_csv(*sample);
  }
}

void SafetySupervisorNode::write_latency_csv(const SafetyLatencySample & sample)
{
  if (!enable_latency_csv_) {
    return;
  }
  latency_csv_ << sample.event_id << ',' << sample.fault_type << ',' << std::fixed << std::setprecision(3) <<
    sample.detection_latency_ms() << ',' << sample.dispatch_latency_ms() << ',' <<
    sample.decision_latency_ms() << ',' << sample.total_latency_ms() << ',' << sample.result_state << '\n';
  latency_csv_.flush();
}

SafetyEvaluation SafetySupervisorNode::evaluate_sources(
  const std::unordered_map<std::string, SafetySourceState> & sources,
  const rclcpp::Time & now) const
{
  // This function reads only the caller-owned snapshot and has no shared-state lock.
  SafetyEvaluation evaluation;
  evaluation.sources_ready = true;
  evaluation.worst_level = SafetyLevel::SAFE;
  evaluation.reason = "all required safety sources are healthy";

  for (const auto & source : required_sources_) {
    const auto & state = sources.at(source);
    if (!state.received) {
      const auto startup_age = (now - started_at_).seconds();
      if (startup_age <= startup_timeout_sec_) {
        evaluation.sources_ready = false;
        evaluation.worst_level = SafetyLevel::UNKNOWN;
        evaluation.reason = "waiting for required safety source: " + source;
        return evaluation;
      }
      evaluation.worst_level = SafetyLevel::FAULT;
      evaluation.reason = "required safety source '" + source + "' did not report before startup timeout";
      return evaluation;
    }
    if (source_is_stale(state, now)) {
      evaluation.worst_level = SafetyLevel::FAULT;
      evaluation.reason = source + " safety status timeout";
      return evaluation;
    }
    if (!state.data_valid) {
      evaluation.worst_level = SafetyLevel::FAULT;
      evaluation.reason = source + " reported invalid safety data: " + state.reason;
      return evaluation;
    }
    if (state.level == SafetyLevel::UNKNOWN) {
      evaluation.worst_level = SafetyLevel::FAULT;
      evaluation.reason = source + " reported an unknown safety level";
      return evaluation;
    }
    const auto previous_worst = evaluation.worst_level;
    evaluation.worst_level = worst_safety_level(evaluation.worst_level, state.level);
    if (evaluation.worst_level != previous_worst) {
      evaluation.reason = source + ": " + state.reason;
    }
  }
  return evaluation;
}

std::size_t SafetySupervisorNode::executor_threads() const noexcept
{
  return executor_threads_;
}

std::uint64_t SafetySupervisorNode::source_callback_count() const noexcept
{
  return source_callback_count_.load();
}

std::uint64_t SafetySupervisorNode::evaluation_callback_count() const noexcept
{
  return evaluation_callback_count_.load();
}

SafetyLevel SafetySupervisorNode::status_level_to_safety_level(std::uint8_t level) const noexcept
{
  switch (level) {
    case SafetyStatus::SAFE:
      return SafetyLevel::SAFE;
    case SafetyStatus::WARNING:
      return SafetyLevel::WARNING;
    case SafetyStatus::STOP:
      return SafetyLevel::STOP;
    case SafetyStatus::ERROR:
      return SafetyLevel::FAULT;
    case SafetyStatus::UNKNOWN:
    default:
      return SafetyLevel::UNKNOWN;
  }
}

std::uint8_t SafetySupervisorNode::state_to_status_level(SystemSafetyState state) const noexcept
{
  switch (state) {
    case SystemSafetyState::SAFE:
      return SafetyStatus::SAFE;
    case SystemSafetyState::WARNING:
      return SafetyStatus::WARNING;
    case SystemSafetyState::STOP:
      return SafetyStatus::STOP;
    case SystemSafetyState::FAULT:
      return SafetyStatus::ERROR;
    case SystemSafetyState::INIT:
    default:
      return SafetyStatus::UNKNOWN;
  }
}

bool SafetySupervisorNode::source_is_stale(
  const SafetySourceState & state, const rclcpp::Time & now) const
{
  // Receipt time catches stopped publishers; message time catches stale source data.
  if ((now - state.last_received).seconds() >= source_timeout_sec_) {
    return true;
  }
  if (!state.has_message_stamp) {
    return false;
  }
  return now < state.message_stamp || (now - state.message_stamp).seconds() >= source_timeout_sec_;
}

void SafetySupervisorNode::publish_system_status(
  SystemSafetyState state, const SafetyEvaluation & evaluation, const rclcpp::Time & now,
  const std::unordered_map<std::string, SafetySourceState> & sources,
  std::int64_t decision_steady_ns)
{
  // Publication happens after snapshot evaluation, never while source_store_ is locked.
  SafetyStatus message;
  message.header.stamp = now;
  message.source = "safety_supervisor";
  message.level = state_to_status_level(state);
  message.data_valid = state != SystemSafetyState::INIT && state != SystemSafetyState::FAULT;
  message.reason = evaluation.reason;
  message.decision_time_steady_ns = decision_steady_ns;

  // Keep the existing Phase 9 correlation visible to a downstream safety
  // consumer. A healthy source's old event is not relevant to a normal state.
  if (state == SystemSafetyState::STOP || state == SystemSafetyState::FAULT) {
    for (const auto & item : sources) {
      const auto & source = item.second;
      if (source.latency_event_id == 0U) {
        continue;
      }
      message.latency_event_id = source.latency_event_id;
      message.latency_fault_type = source.latency_fault_type;
      message.fault_time_steady_ns = source.fault_time_steady_ns;
      message.detection_time_steady_ns = source.detection_time_steady_ns;
      break;
    }
  }
  system_status_publisher_->publish(message);
}

}  // namespace safety_supervisor

#include <chrono>
#include <future>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "arm_navi_safety_interfaces/msg/safety_event.hpp"
#include "arm_navi_safety_interfaces/msg/safety_status.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

namespace
{
using namespace std::chrono_literals;
using arm_navi_safety_interfaces::msg::SafetyEvent;
using arm_navi_safety_interfaces::msg::SafetyStatus;

bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::function<bool()> & predicate,
  const std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  executor.spin_some();
  return predicate();
}

class SafetyMessagesTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(SafetyMessagesTest, ConstantsDefaultsAndFieldAssignment)
{
  EXPECT_EQ(SafetyStatus::UNKNOWN, 0u);
  EXPECT_EQ(SafetyStatus::SAFE, 1u);
  EXPECT_EQ(SafetyStatus::WARNING, 2u);
  EXPECT_EQ(SafetyStatus::STOP, 3u);
  EXPECT_EQ(SafetyStatus::ERROR, 4u);

  EXPECT_EQ(SafetyEvent::UNKNOWN, SafetyStatus::UNKNOWN);
  EXPECT_EQ(SafetyEvent::SAFE, SafetyStatus::SAFE);
  EXPECT_EQ(SafetyEvent::WARNING, SafetyStatus::WARNING);
  EXPECT_EQ(SafetyEvent::STOP, SafetyStatus::STOP);
  EXPECT_EQ(SafetyEvent::ERROR, SafetyStatus::ERROR);

  SafetyStatus status;
  EXPECT_EQ(status.level, SafetyStatus::UNKNOWN);
  EXPECT_TRUE(status.source.empty());
  EXPECT_TRUE(status.reason.empty());
  EXPECT_FALSE(status.data_valid);
  EXPECT_EQ(status.decision_time_steady_ns, 0);

  status.level = SafetyStatus::SAFE;
  status.source = "test";
  status.reason = "unit test";
  status.data_valid = true;
  status.decision_time_steady_ns = 42;
  EXPECT_EQ(status.level, SafetyStatus::SAFE);
  EXPECT_EQ(status.source, "test");
  EXPECT_EQ(status.reason, "unit test");
  EXPECT_TRUE(status.data_valid);
  EXPECT_EQ(status.decision_time_steady_ns, 42);
}

TEST_F(SafetyMessagesTest, InvalidDataScenariosAreRepresentable)
{
  SafetyStatus waiting_for_lidar;
  waiting_for_lidar.level = SafetyStatus::UNKNOWN;
  waiting_for_lidar.source = "lidar_safety";
  waiting_for_lidar.reason = "waiting for lidar data";
  waiting_for_lidar.data_valid = false;

  EXPECT_EQ(waiting_for_lidar.level, SafetyStatus::UNKNOWN);
  EXPECT_EQ(waiting_for_lidar.source, "lidar_safety");
  EXPECT_FALSE(waiting_for_lidar.data_valid);

  SafetyStatus tf_failure;
  tf_failure.level = SafetyStatus::ERROR;
  tf_failure.source = "tf_monitor";
  tf_failure.reason = "transform lookup failed";
  tf_failure.data_valid = false;

  EXPECT_EQ(tf_failure.level, SafetyStatus::ERROR);
  EXPECT_EQ(tf_failure.reason, "transform lookup failed");
  EXPECT_FALSE(tf_failure.data_valid);
}

TEST_F(SafetyMessagesTest, SafetyStatusRoundTripsThroughRosTopic)
{
  auto publisher_node = rclcpp::Node::make_shared("safety_status_test_publisher");
  auto subscriber_node = rclcpp::Node::make_shared("safety_status_test_subscriber");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);

  auto received = std::make_shared<std::promise<SafetyStatus>>();
  auto received_future = received->get_future();
  auto subscription = subscriber_node->create_subscription<SafetyStatus>(
    "/test/safety_status", 10,
    [received](SafetyStatus::SharedPtr message) {
      try {
        received->set_value(*message);
      } catch (const std::future_error &) {
        // The test only needs the first received publication.
      }
    });
  auto publisher = publisher_node->create_publisher<SafetyStatus>("/test/safety_status", 10);

  ASSERT_TRUE(spin_until(
      executor, [&publisher]() {return publisher->get_subscription_count() == 1u;}, 2s));

  SafetyStatus expected;
  expected.header.stamp.sec = 42;
  expected.header.stamp.nanosec = 123456789u;
  expected.header.frame_id = "laser_frame";
  expected.level = SafetyStatus::STOP;
  expected.source = "lidar_safety";
  expected.reason = "test obstacle";
  expected.data_valid = true;
  expected.decision_time_steady_ns = 123456789;
  publisher->publish(expected);

  ASSERT_TRUE(spin_until(
      executor,
      [&received_future]() {return received_future.wait_for(0ms) == std::future_status::ready;},
      2s));
  const auto actual = received_future.get();
  EXPECT_EQ(actual.header.stamp.sec, expected.header.stamp.sec);
  EXPECT_EQ(actual.header.stamp.nanosec, expected.header.stamp.nanosec);
  EXPECT_EQ(actual.header.frame_id, expected.header.frame_id);
  EXPECT_EQ(actual.source, expected.source);
  EXPECT_EQ(actual.level, expected.level);
  EXPECT_EQ(actual.reason, expected.reason);
  EXPECT_EQ(actual.data_valid, expected.data_valid);
  EXPECT_EQ(actual.decision_time_steady_ns, expected.decision_time_steady_ns);
}

TEST_F(SafetyMessagesTest, SafetyEventsRoundTripThroughRosTopic)
{
  auto publisher_node = rclcpp::Node::make_shared("safety_event_test_publisher");
  auto subscriber_node = rclcpp::Node::make_shared("safety_event_test_subscriber");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);

  std::vector<SafetyEvent> received;
  auto subscription = subscriber_node->create_subscription<SafetyEvent>(
    "/test/safety_event", 10,
    [&received](SafetyEvent::SharedPtr message) {received.push_back(*message);});
  auto publisher = publisher_node->create_publisher<SafetyEvent>("/test/safety_event", 10);

  ASSERT_TRUE(spin_until(
      executor, [&publisher]() {return publisher->get_subscription_count() == 1u;}, 2s));

  SafetyEvent safe_to_warning;
  safe_to_warning.header.stamp.sec = 100;
  safe_to_warning.header.frame_id = "laser_frame";
  safe_to_warning.source = "lidar_safety";
  safe_to_warning.previous_level = SafetyEvent::SAFE;
  safe_to_warning.current_level = SafetyEvent::WARNING;
  safe_to_warning.reason = "obstacle entered warning zone";
  safe_to_warning.data_valid = true;

  SafetyEvent warning_to_stop;
  warning_to_stop.header.stamp.sec = 101;
  warning_to_stop.header.frame_id = "laser_frame";
  warning_to_stop.source = "lidar_safety";
  warning_to_stop.previous_level = SafetyEvent::WARNING;
  warning_to_stop.current_level = SafetyEvent::STOP;
  warning_to_stop.reason = "obstacle entered stop zone";
  warning_to_stop.data_valid = true;

  publisher->publish(safe_to_warning);
  publisher->publish(warning_to_stop);
  ASSERT_TRUE(spin_until(executor, [&received]() {return received.size() == 2u;}, 2s));

  EXPECT_EQ(received[0].header.stamp.sec, safe_to_warning.header.stamp.sec);
  EXPECT_EQ(received[0].header.frame_id, safe_to_warning.header.frame_id);
  EXPECT_EQ(received[0].source, safe_to_warning.source);
  EXPECT_EQ(received[0].previous_level, SafetyEvent::SAFE);
  EXPECT_EQ(received[0].current_level, SafetyEvent::WARNING);
  EXPECT_EQ(received[0].reason, "obstacle entered warning zone");
  EXPECT_TRUE(received[0].data_valid);
  EXPECT_EQ(received[1].header.stamp.sec, warning_to_stop.header.stamp.sec);
  EXPECT_EQ(received[1].header.frame_id, warning_to_stop.header.frame_id);
  EXPECT_EQ(received[1].source, warning_to_stop.source);
  EXPECT_EQ(received[1].previous_level, SafetyEvent::WARNING);
  EXPECT_EQ(received[1].current_level, SafetyEvent::STOP);
  EXPECT_EQ(received[1].reason, "obstacle entered stop zone");
  EXPECT_TRUE(received[1].data_valid);
}
}  // namespace

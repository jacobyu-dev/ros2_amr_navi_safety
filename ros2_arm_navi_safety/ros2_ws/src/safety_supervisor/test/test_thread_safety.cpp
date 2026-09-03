#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "safety_supervisor/safety_supervisor_node.hpp"

namespace safety_supervisor
{
namespace
{

TEST(ThreadSafetyTest, AtomicFlagRetainsFinalStore)
{
  std::atomic<bool> emergency_stop{false};
  std::vector<std::thread> workers;
  for (int index = 0; index < 8; ++index) {
    workers.emplace_back([&emergency_stop, index]() {
      for (int iteration = 0; iteration < 10000; ++iteration) {
        emergency_stop.store((iteration + index) % 2 == 0);
      }
    });
  }
  for (auto & worker : workers) {
    worker.join();
  }

  emergency_stop.store(true);
  EXPECT_TRUE(emergency_stop.load());
}

TEST(ThreadSafetyTest, AtomicCounterCountsConcurrentIncrements)
{
  constexpr int kWorkerCount = 8;
  constexpr int kIncrementsPerWorker = 10000;
  std::atomic<std::uint64_t> callback_count{0U};
  std::vector<std::thread> workers;
  for (int index = 0; index < kWorkerCount; ++index) {
    workers.emplace_back([&callback_count]() {
      for (int iteration = 0; iteration < kIncrementsPerWorker; ++iteration) {
        callback_count.fetch_add(1U);
      }
    });
  }
  for (auto & worker : workers) {
    worker.join();
  }

  EXPECT_EQ(callback_count.load(), static_cast<std::uint64_t>(kWorkerCount * kIncrementsPerWorker));
}

TEST(ThreadSafetyTest, SourceSnapshotNeverSplitsCompoundUpdate)
{
  SafetySourceStore store;
  store.initialize({"lidar_safety"});
  std::atomic<bool> writer_finished{false};
  std::atomic<bool> snapshot_consistent{true};

  std::thread writer([&store, &writer_finished]() {
    for (std::uint64_t sequence = 1U; sequence <= 5000U; ++sequence) {
      SafetySourceState state;
      state.received = true;
      state.data_valid = sequence % 2U == 0U;
      state.level = state.data_valid ? SafetyLevel::SAFE : SafetyLevel::WARNING;
      state.reason = std::to_string(sequence);
      store.update("lidar_safety", std::move(state));
    }
    writer_finished.store(true);
  });

  std::thread reader([&store, &writer_finished, &snapshot_consistent]() {
    while (!writer_finished.load()) {
      const auto snapshot = store.snapshot();
      const auto & state = snapshot.at("lidar_safety");
      if (!state.received) {
        continue;
      }
      const auto sequence = std::stoull(state.reason);
      const bool expected_valid = sequence % 2U == 0U;
      if (state.data_valid != expected_valid ||
        state.level != (expected_valid ? SafetyLevel::SAFE : SafetyLevel::WARNING))
      {
        snapshot_consistent.store(false);
        return;
      }
    }
  });

  writer.join();
  reader.join();
  EXPECT_TRUE(snapshot_consistent.load());
  const auto final_state = store.snapshot().at("lidar_safety");
  EXPECT_EQ(final_state.reason, "5000");
  EXPECT_TRUE(final_state.data_valid);
  EXPECT_EQ(final_state.level, SafetyLevel::SAFE);
}

TEST(ThreadSafetyTest, ConcurrentInputsKeepStateMachineInDefinedStates)
{
  SafetySourceStore store;
  store.initialize({"lidar_safety", "tf_monitor"});
  std::atomic<bool> writers_finished{false};
  std::atomic<bool> valid_state{true};
  std::atomic<std::uint64_t> evaluation_count{0U};

  auto write_source = [&store](const std::string & source, SafetyLevel level) {
      for (int iteration = 0; iteration < 2000; ++iteration) {
        SafetySourceState state;
        state.received = true;
        state.data_valid = true;
        state.level = level;
        state.reason = "concurrent input";
        store.update(source, std::move(state));
      }
    };
  std::thread lidar_writer(write_source, "lidar_safety", SafetyLevel::WARNING);
  std::thread tf_writer(write_source, "tf_monitor", SafetyLevel::SAFE);
  std::thread evaluator([&store, &writers_finished, &valid_state, &evaluation_count]() {
    SafetyStateMachine machine;
    while (!writers_finished.load()) {
      const auto snapshot = store.snapshot();
      const auto level = worst_safety_level(
        snapshot.at("lidar_safety").level, snapshot.at("tf_monitor").level);
      const auto state = machine.update(SafetyEvaluation{true, level, "concurrent input"});
      evaluation_count.fetch_add(1U);
      if (state != SystemSafetyState::SAFE && state != SystemSafetyState::WARNING &&
        state != SystemSafetyState::STOP && state != SystemSafetyState::FAULT &&
        state != SystemSafetyState::INIT)
      {
        valid_state.store(false);
        return;
      }
    }
  });

  lidar_writer.join();
  tf_writer.join();
  writers_finished.store(true);
  evaluator.join();
  EXPECT_TRUE(valid_state.load());
  EXPECT_GT(evaluation_count.load(), 0U);
}

}  // namespace
}  // namespace safety_supervisor

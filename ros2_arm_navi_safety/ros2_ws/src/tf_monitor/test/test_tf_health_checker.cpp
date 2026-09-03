#include "tf_monitor/tf_health_checker.hpp"

#include <chrono>
#include <stdexcept>

#include "gtest/gtest.h"

namespace tf_monitor
{
namespace
{
using namespace std::chrono_literals;

TfHealthChecker make_checker()
{
  return TfHealthChecker{TfMonitorConfig{100ms, 100ms, 500ms, true}};
}

TEST(TfHealthCheckerTest, ReportsOkForFreshDynamicTransform)
{
  EXPECT_EQ(make_checker().evaluate(TfObservation{true, false, 100ms}), TfStatus::OK);
}

TEST(TfHealthCheckerTest, ReportsStaleForOldDynamicTransform)
{
  EXPECT_EQ(make_checker().evaluate(TfObservation{true, false, 800ms}), TfStatus::STALE);
}

TEST(TfHealthCheckerTest, TreatsThresholdBoundaryAsOk)
{
  EXPECT_EQ(make_checker().evaluate(TfObservation{true, false, 500ms}), TfStatus::OK);
}

TEST(TfHealthCheckerTest, ReportsMissingWhenNoTransformExists)
{
  EXPECT_EQ(make_checker().evaluate(TfObservation{false, false, 0ms}), TfStatus::MISSING);
}

TEST(TfHealthCheckerTest, MapsLookupFailuresToDistinctStatuses)
{
  EXPECT_EQ(status_from_lookup_failure(TfLookupFailure::MISSING), TfStatus::MISSING);
  EXPECT_EQ(status_from_lookup_failure(TfLookupFailure::TIMEOUT), TfStatus::TIMEOUT);
}

TEST(TfHealthCheckerTest, AcceptsStaticTransformRegardlessOfAge)
{
  EXPECT_EQ(make_checker().evaluate(TfObservation{true, true, 10s}), TfStatus::OK);
}

TEST(TfHealthCheckerTest, RejectsInvalidConfiguration)
{
  EXPECT_THROW(TfHealthChecker(TfMonitorConfig{0ms, 100ms, 500ms, true}), std::invalid_argument);
  EXPECT_THROW(TfHealthChecker(TfMonitorConfig{100ms, 100ms, -1ms, true}), std::invalid_argument);
}

}  // namespace
}  // namespace tf_monitor

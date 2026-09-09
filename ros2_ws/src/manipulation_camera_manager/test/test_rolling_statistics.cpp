#include <gtest/gtest.h>

#include "manipulation_camera_manager/rolling_statistics.hpp"

namespace manipulation_camera_manager {

TEST(RollingStatisticsTest, ComputesPercentilesAndMaximum) {
  RollingStatistics statistics(10U);
  for (int value = 1; value <= 10; ++value) {
    statistics.Add(static_cast<double>(value));
  }
  const auto summary = statistics.Summary();
  EXPECT_EQ(summary.count, 10U);
  EXPECT_DOUBLE_EQ(summary.p50, 5.5);
  EXPECT_DOUBLE_EQ(summary.p95, 9.55);
  EXPECT_DOUBLE_EQ(summary.maximum, 10.0);
}

TEST(RollingStatisticsTest, RetainsOnlyNewestWindow) {
  RollingStatistics statistics(3U);
  statistics.Add(1.0);
  statistics.Add(2.0);
  statistics.Add(3.0);
  statistics.Add(100.0);
  const auto summary = statistics.Summary();
  EXPECT_EQ(summary.count, 3U);
  EXPECT_DOUBLE_EQ(summary.p50, 3.0);
  EXPECT_DOUBLE_EQ(summary.maximum, 100.0);
}

TEST(RollingStatisticsTest, ClearStartsNewPhaseWindow) {
  RollingStatistics statistics(3U);
  statistics.Add(20.0);
  statistics.Clear();
  EXPECT_EQ(statistics.Summary().count, 0U);
}

}  // namespace manipulation_camera_manager

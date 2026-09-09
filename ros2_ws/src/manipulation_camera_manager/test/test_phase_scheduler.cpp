#include <gtest/gtest.h>

#include "manipulation_camera_manager/phase_scheduler.hpp"

namespace manipulation_camera_manager {

TEST(PhaseScheduleTest, NormalizesAndReturnsRules) {
  PhaseSchedule schedule;
  schedule.AddRule("search", "top", CameraPhaseRule{10.0, 8.0});

  ASSERT_TRUE(schedule.HasPhase(" SEARCH "));
  const auto& rule = schedule.Rule("Search", "top");
  EXPECT_DOUBLE_EQ(rule.decode_hz, 10.0);
  EXPECT_DOUBLE_EQ(rule.inference_hz, 8.0);
}

TEST(PhaseScheduleTest, RejectsInvalidRule) {
  PhaseSchedule schedule;
  EXPECT_THROW(
      schedule.AddRule("SEARCH", "top", CameraPhaseRule{-1.0, 0.0}),
      std::invalid_argument);
  EXPECT_THROW(
      schedule.AddRule("SEARCH", "top", CameraPhaseRule{1.0, 2.0}),
      std::invalid_argument);
}

TEST(PhaseScheduleTest, RejectsUnknownPhaseAndCamera) {
  PhaseSchedule schedule;
  schedule.AddRule("STANDBY", "top", CameraPhaseRule{});
  EXPECT_THROW(schedule.Rule("SEARCH", "top"), std::out_of_range);
  EXPECT_THROW(schedule.Rule("STANDBY", "wrist_a"), std::out_of_range);
}

}  // namespace manipulation_camera_manager

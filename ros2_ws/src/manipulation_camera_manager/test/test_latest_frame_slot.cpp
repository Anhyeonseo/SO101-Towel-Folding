#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <vector>

#include "manipulation_camera_manager/latest_frame_slot.hpp"

namespace manipulation_camera_manager {

TEST(LatestFrameSlot, KeepsOnlyNewestFrame) {
  LatestFrameSlot slot;
  const auto start = std::chrono::steady_clock::now();
  slot.Store({1U, 2U}, start, 10U);
  slot.Store({3U, 4U, 5U}, start + std::chrono::milliseconds(20), 11U);

  const auto latest = slot.Snapshot();
  ASSERT_TRUE(latest.has_value());
  EXPECT_EQ(latest->generation, 2U);
  EXPECT_EQ(latest->driver_sequence, 11U);
  EXPECT_EQ(*latest->bytes, (std::vector<std::uint8_t>{3U, 4U, 5U}));
  EXPECT_EQ(slot.ReplacementCount(), 1U);
}

TEST(LatestFrameSlot, ReportsFrameAge) {
  LatestFrameSlot slot;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(slot.Age(start).has_value());
  slot.Store({1U}, start, 1U);
  ASSERT_TRUE(slot.Age(start + std::chrono::milliseconds(75)).has_value());
  EXPECT_EQ(slot.Age(start + std::chrono::milliseconds(75))->count(), 75);
}

}  // namespace manipulation_camera_manager

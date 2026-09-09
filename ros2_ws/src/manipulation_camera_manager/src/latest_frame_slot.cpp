#include "manipulation_camera_manager/latest_frame_slot.hpp"

#include <utility>

namespace manipulation_camera_manager {

void LatestFrameSlot::Store(
    std::vector<std::uint8_t> bytes,
    std::chrono::steady_clock::time_point captured_at,
    std::uint32_t driver_sequence) {
  auto shared_bytes =
      std::make_shared<const std::vector<std::uint8_t>>(std::move(bytes));
  std::lock_guard<std::mutex> lock(mutex_);
  if (latest_.has_value()) {
    ++replacement_count_;
  }
  ++generation_;
  latest_ = CompressedFrame{
      std::move(shared_bytes), captured_at, driver_sequence, generation_};
}

std::optional<CompressedFrame> LatestFrameSlot::Snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

std::optional<std::chrono::milliseconds> LatestFrameSlot::Age(
    std::chrono::steady_clock::time_point now) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_.has_value()) {
    return std::nullopt;
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      now - latest_->captured_at);
}

std::uint64_t LatestFrameSlot::ReplacementCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return replacement_count_;
}

}  // namespace manipulation_camera_manager

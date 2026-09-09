#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

namespace manipulation_camera_manager {

struct CompressedFrame {
  std::shared_ptr<const std::vector<std::uint8_t>> bytes;
  std::chrono::steady_clock::time_point captured_at;
  std::uint32_t driver_sequence{0};
  std::uint64_t generation{0};
};

class LatestFrameSlot {
 public:
  void Store(
      std::vector<std::uint8_t> bytes,
      std::chrono::steady_clock::time_point captured_at,
      std::uint32_t driver_sequence);

  std::optional<CompressedFrame> Snapshot() const;
  std::optional<std::chrono::milliseconds> Age(
      std::chrono::steady_clock::time_point now) const;
  std::uint64_t ReplacementCount() const;

 private:
  mutable std::mutex mutex_;
  std::optional<CompressedFrame> latest_;
  std::uint64_t generation_{0};
  std::uint64_t replacement_count_{0};
};

}  // namespace manipulation_camera_manager

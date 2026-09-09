#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "manipulation_camera_manager/latest_frame_slot.hpp"

namespace manipulation_camera_manager {

struct CameraConfig {
  std::string name;
  std::string device_path;
  std::uint32_t width{640};
  std::uint32_t height{480};
  std::uint32_t fps{30};
  std::uint32_t buffer_count{4};
  bool disable_dynamic_framerate{true};
  std::chrono::milliseconds reconnect_interval{500};
  std::chrono::milliseconds frame_timeout{1500};
};

enum class CameraState : std::uint8_t {
  kDisconnected,
  kConnecting,
  kStreaming,
  kStopped,
};

struct CameraStats {
  CameraState state{CameraState::kDisconnected};
  std::uint64_t frames_received{0};
  std::uint64_t driver_frames_dropped{0};
  std::uint64_t reconnect_count{0};
  std::uint64_t latest_generation{0};
  std::size_t latest_bytes{0};
  std::int64_t latest_age_ms{-1};
  std::string last_error;
};

class V4l2Camera {
 public:
  explicit V4l2Camera(CameraConfig config);
  ~V4l2Camera();

  V4l2Camera(const V4l2Camera&) = delete;
  V4l2Camera& operator=(const V4l2Camera&) = delete;

  void Start();
  void Stop();
  CameraStats Stats() const;
  std::optional<CompressedFrame> LatestFrame() const;
  const CameraConfig& Config() const { return config_; }

 private:
  struct MappedBuffer {
    void* address{nullptr};
    std::size_t length{0};
  };

  void Run();
  void OpenAndConfigure();
  void CaptureUntilFailure();
  void CloseDevice() noexcept;
  void SetError(std::string message);

  CameraConfig config_;
  LatestFrameSlot latest_frame_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<CameraState> state_{CameraState::kStopped};
  std::atomic<std::uint64_t> frames_received_{0};
  std::atomic<std::uint64_t> driver_frames_dropped_{0};
  std::atomic<std::uint64_t> reconnect_count_{0};
  mutable std::mutex error_mutex_;
  std::string last_error_;
  std::thread worker_;
  int fd_{-1};
  bool streaming_{false};
  std::vector<MappedBuffer> mapped_buffers_;
};

const char* CameraStateName(CameraState state);

}  // namespace manipulation_camera_manager

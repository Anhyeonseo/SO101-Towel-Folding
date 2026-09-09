#include "manipulation_camera_manager/v4l2_camera.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <utility>

namespace manipulation_camera_manager {
namespace {

int IoctlRetry(int fd, unsigned long request, void* argument) {
  int result;
  do {
    result = ioctl(fd, request, argument);
  } while (result == -1 && errno == EINTR);
  return result;
}

[[noreturn]] void ThrowErrno(const std::string& operation) {
  throw std::runtime_error(operation + ": " + std::strerror(errno));
}

}  // namespace

const char* CameraStateName(CameraState state) {
  switch (state) {
    case CameraState::kDisconnected:
      return "DISCONNECTED";
    case CameraState::kConnecting:
      return "CONNECTING";
    case CameraState::kStreaming:
      return "STREAMING";
    case CameraState::kStopped:
      return "STOPPED";
  }
  return "UNKNOWN";
}

V4l2Camera::V4l2Camera(CameraConfig config) : config_(std::move(config)) {}

V4l2Camera::~V4l2Camera() { Stop(); }

void V4l2Camera::Start() {
  if (worker_.joinable()) {
    return;
  }
  stop_requested_.store(false);
  worker_ = std::thread(&V4l2Camera::Run, this);
}

void V4l2Camera::Stop() {
  stop_requested_.store(true);
  if (worker_.joinable()) {
    worker_.join();
  }
  CloseDevice();
  state_.store(CameraState::kStopped);
}

CameraStats V4l2Camera::Stats() const {
  CameraStats result;
  result.state = state_.load();
  result.frames_received = frames_received_.load();
  result.driver_frames_dropped = driver_frames_dropped_.load();
  result.reconnect_count = reconnect_count_.load();
  const auto latest = latest_frame_.Snapshot();
  if (latest.has_value()) {
    result.latest_generation = latest->generation;
    result.latest_bytes = latest->bytes->size();
    result.latest_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - latest->captured_at)
                               .count();
  }
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    result.last_error = last_error_;
  }
  return result;
}

std::optional<CompressedFrame> V4l2Camera::LatestFrame() const {
  return latest_frame_.Snapshot();
}

void V4l2Camera::Run() {
  bool connected_once = false;
  while (!stop_requested_.load()) {
    try {
      state_.store(CameraState::kConnecting);
      OpenAndConfigure();
      if (connected_once) {
        reconnect_count_.fetch_add(1);
      }
      connected_once = true;
      SetError("");
      state_.store(CameraState::kStreaming);
      CaptureUntilFailure();
    } catch (const std::exception& error) {
      SetError(error.what());
    }

    CloseDevice();
    if (stop_requested_.load()) {
      break;
    }
    state_.store(CameraState::kDisconnected);
    const auto deadline =
        std::chrono::steady_clock::now() + config_.reconnect_interval;
    while (!stop_requested_.load() &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
  }
  state_.store(CameraState::kStopped);
}

void V4l2Camera::OpenAndConfigure() {
  CloseDevice();
  fd_ = open(config_.device_path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    ThrowErrno("open " + config_.device_path);
  }

  v4l2_capability capability{};
  if (IoctlRetry(fd_, VIDIOC_QUERYCAP, &capability) < 0) {
    ThrowErrno("VIDIOC_QUERYCAP");
  }
  const std::uint32_t caps =
      (capability.capabilities & V4L2_CAP_DEVICE_CAPS) != 0U
          ? capability.device_caps
          : capability.capabilities;
  if ((caps & V4L2_CAP_VIDEO_CAPTURE) == 0U ||
      (caps & V4L2_CAP_STREAMING) == 0U) {
    throw std::runtime_error("device lacks V4L2 capture/streaming capability");
  }

  v4l2_format format{};
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.width = config_.width;
  format.fmt.pix.height = config_.height;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  format.fmt.pix.field = V4L2_FIELD_ANY;
  if (IoctlRetry(fd_, VIDIOC_S_FMT, &format) < 0) {
    ThrowErrno("VIDIOC_S_FMT");
  }
  if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG ||
      format.fmt.pix.width != config_.width ||
      format.fmt.pix.height != config_.height) {
    throw std::runtime_error("camera did not accept requested MJPEG dimensions");
  }

  v4l2_streamparm stream_parameters{};
  stream_parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  stream_parameters.parm.capture.timeperframe.numerator = 1;
  stream_parameters.parm.capture.timeperframe.denominator = config_.fps;
  if (IoctlRetry(fd_, VIDIOC_S_PARM, &stream_parameters) < 0) {
    ThrowErrno("VIDIOC_S_PARM");
  }

  if (config_.disable_dynamic_framerate) {
    v4l2_control exposure_priority{};
    exposure_priority.id = V4L2_CID_EXPOSURE_AUTO_PRIORITY;
    exposure_priority.value = 0;
    // This UVC control is optional. Cameras without it must still capture.
    (void)IoctlRetry(fd_, VIDIOC_S_CTRL, &exposure_priority);
  }

  v4l2_requestbuffers request{};
  request.count = config_.buffer_count;
  request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  request.memory = V4L2_MEMORY_MMAP;
  if (IoctlRetry(fd_, VIDIOC_REQBUFS, &request) < 0) {
    ThrowErrno("VIDIOC_REQBUFS");
  }
  if (request.count < 2U) {
    throw std::runtime_error("camera returned fewer than two mmap buffers");
  }

  mapped_buffers_.resize(request.count);
  for (std::uint32_t index = 0; index < request.count; ++index) {
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;
    if (IoctlRetry(fd_, VIDIOC_QUERYBUF, &buffer) < 0) {
      ThrowErrno("VIDIOC_QUERYBUF");
    }
    void* address = mmap(
        nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_,
        buffer.m.offset);
    if (address == MAP_FAILED) {
      ThrowErrno("mmap");
    }
    mapped_buffers_[index] = MappedBuffer{address, buffer.length};
    if (IoctlRetry(fd_, VIDIOC_QBUF, &buffer) < 0) {
      ThrowErrno("VIDIOC_QBUF initial");
    }
  }

  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (IoctlRetry(fd_, VIDIOC_STREAMON, &type) < 0) {
    ThrowErrno("VIDIOC_STREAMON");
  }
  streaming_ = true;
}

void V4l2Camera::CaptureUntilFailure() {
  auto last_frame_at = std::chrono::steady_clock::now();
  bool have_sequence = false;
  std::uint32_t last_sequence = 0;

  while (!stop_requested_.load()) {
    pollfd descriptor{};
    descriptor.fd = fd_;
    descriptor.events = POLLIN | POLLPRI;
    const int poll_result = poll(&descriptor, 1, 250);
    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      ThrowErrno("poll");
    }
    if (poll_result == 0) {
      if (std::chrono::steady_clock::now() - last_frame_at >
          config_.frame_timeout) {
        throw std::runtime_error("frame timeout");
      }
      continue;
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      throw std::runtime_error("camera poll reported disconnect/error");
    }

    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (IoctlRetry(fd_, VIDIOC_DQBUF, &buffer) < 0) {
      if (errno == EAGAIN) {
        continue;
      }
      ThrowErrno("VIDIOC_DQBUF");
    }
    if (buffer.index >= mapped_buffers_.size() ||
        buffer.bytesused > mapped_buffers_[buffer.index].length) {
      throw std::runtime_error("camera returned an invalid buffer");
    }

    if (have_sequence) {
      const std::uint32_t difference = buffer.sequence - last_sequence;
      if (difference > 1U && difference < 0x80000000U) {
        driver_frames_dropped_.fetch_add(difference - 1U);
      }
    }
    have_sequence = true;
    last_sequence = buffer.sequence;
    last_frame_at = std::chrono::steady_clock::now();

    const auto* begin = static_cast<const std::uint8_t*>(
        mapped_buffers_[buffer.index].address);
    std::vector<std::uint8_t> frame(begin, begin + buffer.bytesused);
    latest_frame_.Store(std::move(frame), last_frame_at, buffer.sequence);
    frames_received_.fetch_add(1);

    if (IoctlRetry(fd_, VIDIOC_QBUF, &buffer) < 0) {
      ThrowErrno("VIDIOC_QBUF");
    }
  }
}

void V4l2Camera::CloseDevice() noexcept {
  if (fd_ >= 0 && streaming_) {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    (void)IoctlRetry(fd_, VIDIOC_STREAMOFF, &type);
  }
  streaming_ = false;
  for (const auto& buffer : mapped_buffers_) {
    if (buffer.address != nullptr && buffer.address != MAP_FAILED) {
      (void)munmap(buffer.address, buffer.length);
    }
  }
  mapped_buffers_.clear();
  if (fd_ >= 0) {
    (void)close(fd_);
    fd_ = -1;
  }
}

void V4l2Camera::SetError(std::string message) {
  std::lock_guard<std::mutex> lock(error_mutex_);
  last_error_ = std::move(message);
}

}  // namespace manipulation_camera_manager

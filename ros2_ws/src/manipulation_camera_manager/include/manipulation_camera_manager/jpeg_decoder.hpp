#pragma once

#include <cstdint>
#include <vector>

namespace manipulation_camera_manager {

struct DecodedImage {
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::vector<std::uint8_t> rgb;
};

DecodedImage DecodeJpeg(const std::vector<std::uint8_t>& jpeg);

}  // namespace manipulation_camera_manager

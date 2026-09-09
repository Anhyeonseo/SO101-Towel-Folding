#include <gtest/gtest.h>

#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "manipulation_camera_manager/jpeg_decoder.hpp"

extern "C" {
#include <jpeglib.h>
}

namespace manipulation_camera_manager {
namespace {

std::vector<std::uint8_t> MakeTwoPixelJpeg() {
  jpeg_compress_struct compressor{};
  jpeg_error_mgr error{};
  compressor.err = jpeg_std_error(&error);
  jpeg_create_compress(&compressor);
  unsigned char* output = nullptr;
  unsigned long output_size = 0;
  jpeg_mem_dest(&compressor, &output, &output_size);
  compressor.image_width = 2;
  compressor.image_height = 1;
  compressor.input_components = 3;
  compressor.in_color_space = JCS_RGB;
  jpeg_set_defaults(&compressor);
  jpeg_set_quality(&compressor, 100, TRUE);
  jpeg_start_compress(&compressor, TRUE);
  std::uint8_t pixels[] = {255U, 0U, 0U, 0U, 255U, 0U};
  JSAMPROW row = pixels;
  (void)jpeg_write_scanlines(&compressor, &row, 1U);
  jpeg_finish_compress(&compressor);
  std::vector<std::uint8_t> result(output, output + output_size);
  jpeg_destroy_compress(&compressor);
  std::free(output);
  return result;
}

}  // namespace

TEST(JpegDecoderTest, DecodesRgbDimensionsAndBytes) {
  const auto decoded = DecodeJpeg(MakeTwoPixelJpeg());
  EXPECT_EQ(decoded.width, 2U);
  EXPECT_EQ(decoded.height, 1U);
  EXPECT_EQ(decoded.rgb.size(), 6U);
}

TEST(JpegDecoderTest, RejectsEmptyAndTruncatedInput) {
  EXPECT_THROW(DecodeJpeg({}), std::runtime_error);
  const std::vector<std::uint8_t> truncated{0xFFU, 0xD8U, 0xFFU, 0xD9U};
  EXPECT_THROW(DecodeJpeg(truncated), std::runtime_error);
}

}  // namespace manipulation_camera_manager

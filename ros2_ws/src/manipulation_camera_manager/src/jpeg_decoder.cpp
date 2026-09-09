#include "manipulation_camera_manager/jpeg_decoder.hpp"

#include <csetjmp>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>

extern "C" {
#include <jpeglib.h>
}

namespace manipulation_camera_manager {
namespace {

struct JpegErrorManager {
  jpeg_error_mgr base;
  std::jmp_buf jump_buffer;
  char message[JMSG_LENGTH_MAX]{};
  std::uint8_t* allocated{nullptr};
};

void JpegErrorExit(j_common_ptr common) {
  auto* error = reinterpret_cast<JpegErrorManager*>(common->err);
  error->base.format_message(common, error->message);
  std::longjmp(error->jump_buffer, 1);
}

void SuppressJpegMessage(j_common_ptr) {}

}  // namespace

DecodedImage DecodeJpeg(const std::vector<std::uint8_t>& jpeg) {
  if (jpeg.size() < 4U) {
    throw std::runtime_error("JPEG frame is too small");
  }

  jpeg_decompress_struct decoder{};
  auto error = std::make_unique<JpegErrorManager>();
  decoder.err = jpeg_std_error(&error->base);
  error->base.error_exit = JpegErrorExit;
  error->base.output_message = SuppressJpegMessage;
  if (setjmp(error->jump_buffer) != 0) {
    std::free(error->allocated);
    jpeg_destroy_decompress(&decoder);
    throw std::runtime_error(
        std::string("JPEG decode failed: ") + error->message);
  }

  jpeg_create_decompress(&decoder);
  jpeg_mem_src(
      &decoder, jpeg.data(), static_cast<unsigned long>(jpeg.size()));
  (void)jpeg_read_header(&decoder, TRUE);
  decoder.out_color_space = JCS_RGB;
  (void)jpeg_start_decompress(&decoder);
  if (decoder.output_width == 0U || decoder.output_height == 0U ||
      decoder.output_width > 4096U || decoder.output_height > 4096U ||
      decoder.output_components != 3U) {
    jpeg_destroy_decompress(&decoder);
    throw std::runtime_error("JPEG dimensions or color format are invalid");
  }

  const std::uint32_t width = decoder.output_width;
  const std::uint32_t height = decoder.output_height;
  const std::size_t row_bytes =
      static_cast<std::size_t>(width) * 3U;
  const std::size_t image_bytes = row_bytes * height;
  error->allocated = static_cast<std::uint8_t*>(std::malloc(image_bytes));
  if (error->allocated == nullptr) {
    jpeg_destroy_decompress(&decoder);
    throw std::bad_alloc();
  }
  while (decoder.output_scanline < decoder.output_height) {
    JSAMPROW row =
        error->allocated + decoder.output_scanline * row_bytes;
    (void)jpeg_read_scanlines(&decoder, &row, 1U);
  }
  (void)jpeg_finish_decompress(&decoder);
  jpeg_destroy_decompress(&decoder);
  DecodedImage image;
  image.width = width;
  image.height = height;
  try {
    image.rgb.resize(image_bytes);
  } catch (...) {
    std::free(error->allocated);
    error->allocated = nullptr;
    throw;
  }
  std::memcpy(image.rgb.data(), error->allocated, image_bytes);
  std::free(error->allocated);
  error->allocated = nullptr;
  return image;
}

}  // namespace manipulation_camera_manager

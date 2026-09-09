cmake_minimum_required(VERSION 3.16)
project(manipulation_camera_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(JPEG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(camera_capture_core
  src/latest_frame_slot.cpp
  src/v4l2_camera.cpp
)
target_include_directories(camera_capture_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(camera_capture_core PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(camera_capture_core Threads::Threads)

add_library(camera_processing_core
  src/jpeg_decoder.cpp
  src/phase_scheduler.cpp
  src/rolling_statistics.cpp
)
target_include_directories(camera_processing_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(camera_processing_core PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(camera_processing_core JPEG::JPEG)

add_executable(camera_manager_node src/camera_manager_node.cpp)
target_link_libraries(
  camera_manager_node camera_capture_core camera_processing_core)
ament_target_dependencies(
  camera_manager_node diagnostic_msgs rclcpp sensor_msgs std_msgs)
target_compile_options(camera_manager_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS camera_capture_core camera_processing_core camera_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_latest_frame_slot test/test_latest_frame_slot.cpp)
  if(TARGET test_latest_frame_slot)
    target_link_libraries(test_latest_frame_slot camera_capture_core)
  endif()

  ament_add_gtest(test_phase_scheduler test/test_phase_scheduler.cpp)
  if(TARGET test_phase_scheduler)
    target_link_libraries(test_phase_scheduler camera_processing_core)
  endif()

  ament_add_gtest(test_rolling_statistics test/test_rolling_statistics.cpp)
  if(TARGET test_rolling_statistics)
    target_link_libraries(test_rolling_statistics camera_processing_core)
  endif()

  ament_add_gtest(test_jpeg_decoder test/test_jpeg_decoder.cpp)
  if(TARGET test_jpeg_decoder)
    target_link_libraries(test_jpeg_decoder camera_processing_core)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(camera_capture_core camera_processing_core)
ament_package()

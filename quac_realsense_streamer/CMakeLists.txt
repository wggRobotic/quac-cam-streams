cmake_minimum_required(VERSION 3.18)
project(quac_realsense_streamer LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


##################################################
# realsense_streamer
##################################################

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs)
find_package(sensor_msgs)
find_package(geometry_msgs REQUIRED)
find_package(quac_interfaces REQUIRED)

add_executable(realsense_streamer
  src/realsense_streamer.cpp
)

target_include_directories(realsense_streamer PRIVATE
  ${GST_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIR}
  ${OPENCV_INCLUDE_DIRS}
  include/
)

target_link_libraries(realsense_streamer
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
  ${realsense2_LIBRARY}
)

ament_target_dependencies(realsense_streamer
  rclcpp
  std_msgs
  sensor_msgs
  quac_interfaces
)

install(
  TARGETS realsense_streamer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
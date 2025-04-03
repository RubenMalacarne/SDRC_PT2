cmake_minimum_required(VERSION 3.8)
project(vision_localizer)

# ------------------------------------------------------------------------------
# Compiler options
# ------------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ------------------------------------------------------------------------------
# Include directories
# ------------------------------------------------------------------------------
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------
# Executable node
# ------------------------------------------------------------------------------
add_executable(object_localizer
  src/object_localizer_main.cpp
  src/object_localizer.cpp
)

ament_target_dependencies(object_localizer
  cv_bridge
  geometry_msgs
  OpenCV
  rclcpp
  sensor_msgs
  tf2_ros
)

# ------------------------------------------------------------------------------
# Install targets and resources
# ------------------------------------------------------------------------------
install(TARGETS object_localizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Package export
# ------------------------------------------------------------------------------
ament_package()

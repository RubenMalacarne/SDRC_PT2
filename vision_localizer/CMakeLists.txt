cmake_minimum_required(VERSION 3.8)
project(vision_localizer)

# -------------------------------------------------------------------
# Compiler flags
# -------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------------
# Find dependencies
# -------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# -------------------------------------------------------------------
# Message generation
# -------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectInfo.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

# -------------------------------------------------------------------
# Include paths
# -------------------------------------------------------------------
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# -------------------------------------------------------------------
# Executable
# -------------------------------------------------------------------
add_executable(object_localizer
  src/object_localizer_main.cpp
  src/object_localizer.cpp
)

ament_target_dependencies(object_localizer
  rclcpp
  sensor_msgs
  geometry_msgs
  builtin_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  OpenCV
)

# ✨ Link messaggi custom al nodo
rosidl_target_interfaces(object_localizer
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# -------------------------------------------------------------------
# Install
# -------------------------------------------------------------------
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

# -------------------------------------------------------------------
# Final export
# -------------------------------------------------------------------
ament_package()

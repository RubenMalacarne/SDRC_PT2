cmake_minimum_required(VERSION 3.8)
project(scene_management)

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
find_package(interfaces REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# -------------------------------------------------------------------
# Executable
# -------------------------------------------------------------------
add_executable(object_spawner
  src/object_spawner_main.cpp
  src/object_spawner.cpp
)

target_include_directories(object_spawner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(object_spawner
  ament_cmake
  interfaces
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  rclcpp
)

# -------------------------------------------------------------------
# Install
# -------------------------------------------------------------------
install(TARGETS object_spawner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------------------------------------------------
# Final export
# -------------------------------------------------------------------
ament_package()

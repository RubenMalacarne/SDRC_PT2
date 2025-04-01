cmake_minimum_required(VERSION 3.10)
project(coppelia_pick_and_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
# find_package(moveit_task_constructor_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(coppelia_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


add_library(test_action_server SHARED src/test_action_server.cpp)
target_include_directories(test_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(test_action_server
  PRIVATE "TASK_SERVER_CPP_BUILDING_DLL")
ament_target_dependencies(test_action_server
  "coppelia_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface")
rclcpp_components_register_node(test_action_server 
  PLUGIN "coppelia_pick_and_place::TestActionServer" 
  EXECUTABLE test_action_server_node
)

# Motion Action Server
add_library(motion_action_server SHARED src/motion_action_server.cpp)
target_include_directories(motion_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(motion_action_server
  PRIVATE "TASK_SERVER_CPP_BUILDING_DLL")
ament_target_dependencies(motion_action_server
  "coppelia_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface")
rclcpp_components_register_node(motion_action_server 
  PLUGIN "coppelia_pick_and_place::TestActionServer" 
  EXECUTABLE motion_action_server_node
)

# Install the library
install(TARGETS 
  test_action_server
  motion_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(test_mini  src/test_mini.cpp)
target_include_directories(test_mini PUBLIC include)
  ament_target_dependencies(
    test_mini 
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs
  )

  add_executable(scena src/scena.cpp)
  ament_target_dependencies(scena
    rclcpp
    moveit_msgs
    geometry_msgs
    shape_msgs
    geometric_shapes
  )


install(TARGETS
  test_mini
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS scena
  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
  


ament_package()

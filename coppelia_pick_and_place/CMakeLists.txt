cmake_minimum_required(VERSION 3.10)
project(coppelia_pick_and_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------ Dependencies ------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(coppelia_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ------------------ ACTION SERVER ------------------
add_library(test_action_server SHARED src/test_action_server.cpp)
target_include_directories(test_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(test_action_server PRIVATE "TASK_SERVER_CPP_BUILDING_DLL")
ament_target_dependencies(test_action_server
  rclcpp rclcpp_action rclcpp_components
  moveit_ros_planning_interface
  coppelia_msgs)

rclcpp_components_register_node(test_action_server 
  PLUGIN "coppelia_pick_and_place::TestActionServer" 
  EXECUTABLE test_action_server_node
)

# ------------------ MOVE ACTION SERVER ------------------
add_library(move_action SHARED src/move_action.cpp)
target_include_directories(move_action PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(move_action PRIVATE "MOVE_ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(move_action
  rclcpp rclcpp_action rclcpp_components
  moveit_ros_planning_interface
  coppelia_msgs)

rclcpp_components_register_node(move_action
  PLUGIN "coppelia_pick_and_place::MoveActionServer"
  EXECUTABLE move_action_node
)

# ------------------ ACTION CLIENT ------------------
add_library(move_action_client_lib SHARED src/move_action_client.cpp)
target_include_directories(move_action_client_lib PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(move_action_client_lib PRIVATE "SIMPLE_ACTION_CLIENT_CPP_BUILDING_DLL")
ament_target_dependencies(move_action_client_lib
  rclcpp rclcpp_action rclcpp_components
  coppelia_msgs geometry_msgs)

rclcpp_components_register_node(move_action_client_lib 
  PLUGIN "coppelia_pick_and_place::MoveActionClient" 
  EXECUTABLE move_action_client_node
)

# ------------------ ESEGUIBILI NORMALI ------------------
add_executable(test_mini src/test_mini.cpp)
target_include_directories(test_mini PUBLIC include)
ament_target_dependencies(test_mini
  rclcpp rclcpp_action
  moveit_core moveit_ros_planning moveit_ros_planning_interface
  interactive_markers control_msgs)

add_executable(scena src/scena.cpp)
ament_target_dependencies(scena
  rclcpp moveit_msgs geometry_msgs shape_msgs geometric_shapes)

# ------------------ INSTALL ------------------
install(TARGETS
  test_action_server
  test_action_server_node
  move_action
  move_action_node
  move_action_client_lib
  move_action_client_node
  test_mini
  scena
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ------------------ PACKAGE ------------------
ament_package()

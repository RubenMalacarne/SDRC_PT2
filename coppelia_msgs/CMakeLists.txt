cmake_minimum_required(VERSION 3.8)
project(coppelia_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CoppeliaTask.action"
  "msg/DetectedSurfaces.msg"
  "msg/DetectedObjects.msg"
  "msg/InferenceResult.msg"
  "msg/Yolov8Inference.msg"
  "msg/PoseKeypoint.msg"
  "msg/PoseResult.msg"
  "msg/SegmentationResult.msg"
  "msg/Yolov8Segmentation.msg"
  "msg/ObjectDetectionBox.msg"
  "msg/ObjectDetectionResult.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs

)
ament_package()

cmake_minimum_required(VERSION 3.16)
project(custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Suppress developer warnings
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
# for mac python conda, comment if don't need
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DrivetrainControl.msg"
  "msg/DrivetrainFeedback.msg"
  "msg/ScienceFeedback.msg"
  "msg/ScienceControl.msg"
  "msg/ESP32Control.msg"
  "msg/ESP32Sensors.msg"
  "msg/RadioFeedback.msg"
  "msg/BatteryInfo.msg"
  "msg/BmsStatus.msg"
  "msg/CoreControl.msg"
  "msg/CoreFeedback.msg"
  "msg/ArmIncrementCommand.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

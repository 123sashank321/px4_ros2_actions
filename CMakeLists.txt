cmake_minimum_required(VERSION 3.8)
project(px4_ros2_actions_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)

set(action_files
  "action/Arm.action"
  "action/Disarm.action"
  "action/ExecuteMission.action"
  "action/GoToPosition.action"
  "action/Land.action"
  "action/SetMode.action"
  "action/Takeoff.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES action_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Executables
add_executable(arm_disarm_server src/arm_disarm_server.cpp)
ament_target_dependencies(arm_disarm_server rclcpp rclcpp_action px4_msgs action_msgs)
target_link_libraries(arm_disarm_server "${cpp_typesupport_target}")

add_executable(takeoff_land_server src/takeoff_land_server.cpp)
ament_target_dependencies(takeoff_land_server rclcpp rclcpp_action px4_msgs action_msgs)
target_link_libraries(takeoff_land_server "${cpp_typesupport_target}")

add_executable(goto_position_server src/goto_position_server.cpp)
ament_target_dependencies(goto_position_server rclcpp rclcpp_action px4_msgs action_msgs)
target_link_libraries(goto_position_server "${cpp_typesupport_target}")

add_executable(set_mode_server src/set_mode_server.cpp)
ament_target_dependencies(set_mode_server rclcpp rclcpp_action px4_msgs action_msgs)
target_link_libraries(set_mode_server "${cpp_typesupport_target}")

add_executable(mission_executor_server src/mission_executor_server.cpp)
ament_target_dependencies(mission_executor_server rclcpp rclcpp_action px4_msgs action_msgs)
target_link_libraries(mission_executor_server "${cpp_typesupport_target}" yaml-cpp)

install(TARGETS
  arm_disarm_server
  takeoff_land_server
  goto_position_server
  set_mode_server
  mission_executor_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  missions
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

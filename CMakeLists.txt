cmake_minimum_required(VERSION 3.8)
project(px4_ros2_actions)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Takeoff.action"
  "action/Arm.action"
  "action/Disarm.action"
  "action/Land.action"
  "action/RTL.action"
  "action/GoToPosition.action"
  "action/ExecuteMission.action"
  "action/SetMode.action"
  DEPENDENCIES px4_msgs action_msgs
)

# ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  px4_ros2_actions/takeoff_action_server.py
  px4_ros2_actions/land_action_server.py
  px4_ros2_actions/arm_disarm_server.py
  px4_ros2_actions/goto_position_server.py
  px4_ros2_actions/mission_executor_server.py
  px4_ros2_actions/set_mode_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install vehicle_config.py alongside executables
install(FILES
  px4_ros2_actions/vehicle_config.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install vehicle_strategies directory alongside executables
install(DIRECTORY px4_ros2_actions/vehicle_strategies
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY missions
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

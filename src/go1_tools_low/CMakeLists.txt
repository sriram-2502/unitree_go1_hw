cmake_minimum_required(VERSION 3.8)
project(go1_tools_low)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Unitree SDK
set(SDK_DIR /home/sriramk/unitree_ws/src/unitree_ros2/unitree_legged_real/unitree_legged_sdk-master)

if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

set(EXTRA_LIBS -pthread libunitree_legged_sdk_${ARCH}.so lcm)

include_directories(
  include
  ${SDK_DIR}/include
)

link_directories(${SDK_DIR}/lib)

install(
  DIRECTORY ${SDK_DIR}/lib/
  DESTINATION lib
)

add_executable(ros2_torque_example src/ros2_torque_example.cpp)
target_link_libraries(ros2_torque_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_torque_example rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_position_example src/ros2_position_example.cpp)
target_link_libraries(ros2_position_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_position_example rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_standup_controller src/ros2_standup_controller.cpp)
target_link_libraries(ros2_standup_controller ${EXTRA_LIBS})
ament_target_dependencies(ros2_standup_controller rclcpp ros2_unitree_legged_msgs geometry_msgs)

install(TARGETS
  ros2_torque_example
  ros2_position_example
  ros2_standup_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/go1_low_state_converter.py
  scripts/srb_velocity_estimator.py
  scripts/pympc_pd_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch + config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

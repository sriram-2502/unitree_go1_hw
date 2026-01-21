cmake_minimum_required(VERSION 3.8)
project(go1_tools_high)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)

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

install(PROGRAMS
  scripts/go1_high_state_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(ros2_highcmd_profile src/ros2_highcmd_profile.cpp)
target_link_libraries(ros2_highcmd_profile ${EXTRA_LIBS})
ament_target_dependencies(ros2_highcmd_profile rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_highcmd_loop src/ros2_highcmd_loop.cpp)
target_link_libraries(ros2_highcmd_loop ${EXTRA_LIBS})
ament_target_dependencies(ros2_highcmd_loop rclcpp ros2_unitree_legged_msgs geometry_msgs)

install(TARGETS
  ros2_highcmd_profile
  ros2_highcmd_loop
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

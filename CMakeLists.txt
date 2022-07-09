cmake_minimum_required(VERSION 3.5)
project(cyberdog_utils)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(motion_msgs REQUIRED)
find_package(interaction_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include)

add_executable(rs_follower src/follower.cpp)
ament_target_dependencies(rs_follower
  interaction_msgs
  motion_msgs
  image_geometry
  std_srvs
  geometry_msgs
  rclcpp
  rclcpp_action
  sensor_msgs)

install(TARGETS rs_follower
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/text_to_sound.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

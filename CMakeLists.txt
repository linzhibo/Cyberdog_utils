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
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(motion_msgs REQUIRED)

include_directories(include)

add_executable(rs_follower src/follower.cpp)
ament_target_dependencies(rs_follower
  "motion_msgs"
  "image_geometry"
  "geometry_msgs"
  "rclcpp"
  "sensor_msgs")

install(TARGETS rs_follower
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

        

ament_package()

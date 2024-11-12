cmake_minimum_required(VERSION 3.5)
project(gazebo_pose_publisher)

set(ROS_DISTRO $ENV{ROS_DISTRO})

set(PLUGIN_LIB_NAME "gazebo_pose_publisher")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PLUGIN_LIB_NAME} SHARED src/pose_publisher.cpp)

ament_target_dependencies(${PLUGIN_LIB_NAME}
  rclcpp
  geometry_msgs
  gazebo_ros
)

target_link_libraries(${PLUGIN_LIB_NAME} ${GAZEBO_LIBRARIES})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")
install(TARGETS ${PLUGIN_LIB_NAME} DESTINATION lib)

ament_package()

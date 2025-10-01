cmake_minimum_required(VERSION 3.5)
project(virtual_camera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 pacakges
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)

# external dependencies
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBRARIES})

add_executable(streamputer src/streamputer.cpp)
ament_target_dependencies(streamputer rclcpp cv_bridge image_transport sensor_msgs)

install(TARGETS streamputer DESTINATION lib/${PROJECT_NAME})

file(GLOB SCRIPT_FILES scripts/*)
install(PROGRAMS ${SCRIPT_FILES} DESTINATION lib/${PROJECT_NAME})

ament_package()


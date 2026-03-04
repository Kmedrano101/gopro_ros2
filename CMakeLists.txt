cmake_minimum_required(VERSION 3.8)
project(gopro_ros2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -O3)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# System dependencies
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio imgcodecs)
find_package(CURL REQUIRED)

set(ROS_DEPS
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  rclcpp_lifecycle
)

# GoPro camera node
add_executable(gopro_camera_node
  src/gopro_camera_node.cpp
  src/gopro_stream.cpp
)

target_include_directories(gopro_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gopro_camera_node ${ROS_DEPS})

target_link_libraries(gopro_camera_node
  ${OpenCV_LIBS}
  CURL::libcurl
  pthread
)

# Install
install(TARGETS gopro_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

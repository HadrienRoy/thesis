cmake_minimum_required(VERSION 3.5)
project(picam_node)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(DEPENDENCIES
  "rclcpp"
  "sensor_msgs"
)

add_executable( camera_info_node src/camera_info.cpp )
ament_target_dependencies( camera_info_node ${DEPENDENCIES} )


install(TARGETS
  camera_info_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

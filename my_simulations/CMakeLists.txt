cmake_minimum_required(VERSION 3.5)
project(my_simulations)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(docking_interfaces REQUIRED)



### BUILD ###
set(DEPENDENCIES
  "rclcpp"
  "sensor_msgs"
  "docking_interfaces"
  "geometry_msgs"
)


### INSTALL ###

install(DIRECTORY 
  launch
  models
  rviz
  src
  worlds
  DESTINATION share/${PROJECT_NAME}
)

add_executable( gazebo_battery_state src/gazebo_battery_state.cpp )
ament_target_dependencies( gazebo_battery_state ${DEPENDENCIES} )

install(TARGETS
  gazebo_battery_state
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(docking_interfaces)
ament_package()
cmake_minimum_required(VERSION 3.5)
project(onix_robot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(onix_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


## COMPILE
add_library(
  onix_hardware
  SHARED
  src/onix_hardware.cpp
  src/onix_hardware_interface.cpp
)

target_include_directories(
  onix_hardware
  PRIVATE
  include
)

ament_target_dependencies(
  onix_hardware
  onix_msgs
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface onix_hardware.xml)

# Install Python modules

ament_python_install_package(${PROJECT_NAME})

# INSTALL
install(
  TARGETS onix_hardware
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/install
           scripts/uninstall
           onix_robot/diagnostics_updater
  DESTINATION lib/${PROJECT_NAME}
)


## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  onix_hardware
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
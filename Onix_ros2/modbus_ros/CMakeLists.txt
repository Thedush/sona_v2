cmake_minimum_required(VERSION 3.5)
project(modbus_ros)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(agv_srv REQUIRED)

FIND_LIBRARY(mylib_LIBRARIES libmodbus.so.5 /usr/lib/x86_64-linux-gnu/)
include_directories(
  include
)





add_executable(modbus_test src/mod1.cpp)

ament_target_dependencies(modbus_test rclcpp std_msgs)
target_link_libraries(modbus_test ${mylib_LIBRARIES})

install(
  TARGETS modbus_test
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


add_executable(modbus_server src/modbus_server.cpp src/modbus_main.cpp  )

ament_target_dependencies(modbus_server rclcpp std_msgs agv_srv)
target_link_libraries(modbus_server ${mylib_LIBRARIES})

install(
  TARGETS modbus_server
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
include/
DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

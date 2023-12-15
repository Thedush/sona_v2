cmake_minimum_required(VERSION 3.5)
project(modbus_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(onix_msgs REQUIRED)

FIND_LIBRARY(mylib_LIBRARIES libmodbus.so.5 /usr/lib/x86_64-linux-gnu/)
include_directories(
  include
)


add_executable(modbus_driver src/main.cpp src/driver.cpp)

ament_target_dependencies(modbus_driver rclcpp std_msgs onix_msgs)
target_link_libraries(modbus_driver ${mylib_LIBRARIES})

add_executable(rfid_modbus_driver src/rfid_main.cpp src/rfid_node.cpp)

ament_target_dependencies(rfid_modbus_driver rclcpp std_msgs onix_msgs)
target_link_libraries(rfid_modbus_driver ${mylib_LIBRARIES})



install(
  TARGETS modbus_driver
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)



install(
  TARGETS rfid_modbus_driver
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
include/
DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

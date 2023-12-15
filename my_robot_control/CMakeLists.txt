cmake_minimum_required(VERSION 3.5)
project(my_robot_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(onix_msgs REQUIRED)
# Build the executable
add_executable(${PROJECT_NAME} src/my_robot_control.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs geometry_msgs onix_msgs )

# Include directories
target_include_directories(${PROJECT_NAME}
    PRIVATE
        include
)

# Link against the required libraries
# target_link_libraries(${PROJECT_NAME}
#     PRIVATE
#         rclcpp
#         geometry_msgs
# )
set(DEPENDENCIES
  "rclcpp"           
  "std_msgs"         
  "geometry_msgs"
)

# Install the executable
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
# install(DIRECTORY
#     launch
#     DESTINATION share/${PROJECT_NAME}
# )

# Install package.xml
install(FILES
    package.xml
    DESTINATION share/${PROJECT_NAME}
)

# Ament package generation macro
ament_package()
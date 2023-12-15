# Use an official ROS2 image as the base image
FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends apt-utils
RUN apt-get install libpcap-dev -y && apt-get install libmodbus5
# Set the working directory in the container to /root
WORKDIR /root

RUN mkdir -p modbus_ws/src



# install process

# # Copy the local ROS2 package to the container
# # COPY src /root/ls_ws/src

# # Set the working directory to the copied package
# WORKDIR /root/ls_ws

# RUN source /opt/ros/foxy/setup.bash \
#     && rosdep update \
#     && rosdep install -y -r --from-paths src --ignore-src --rosdistro=foxy -y

# RUN source /opt/ros/foxy/setup.bash \
#     && colcon build  --cmake-args=-DCMAKE_BUILD_TYPE=Release \
#     # && colcon test

# # Build the package using colcon
# # RUN colcon build --symlink-install || echo "Installation of package1 failed, continuing with the next command."

# # Delete the source files while keeping the executables
# # RUN rm -rf /root/my_package/src

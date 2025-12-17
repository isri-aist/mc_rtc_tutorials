# Indicates docker image
FROM osrf/ros:humble-desktop-full

# Configure environment
ENV DISPLAY host.docker.internal:0.0

# Copy files
COPY ./ /root/colcon_ws/src/

# Set default bash shell
SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y curl

RUN curl -1sLf 'https://dl.cloudsmith.io/public/mc-rtc/head/setup.deb.sh' | bash

RUN apt install -y libmc-rtc-dev mc-rtc-utils

RUN apt install -y ros-${ROS_DISTRO}-mc-rtc-plugin ros-${ROS_DISTRO}-mc-rtc-tools

# Set start up directory
WORKDIR /root/colcon_ws

# Set environment variables (for using commands)
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
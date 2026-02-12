# Indicates docker image
FROM osrf/ros:humble-desktop-full

# Configure environment
ENV DISPLAY host.docker.internal:0.0

# Set default bash shell
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt install -y curl ca-certificates gpg wget

RUN curl -1sLf 'https://dl.cloudsmith.io/public/mc-rtc/head/setup.deb.sh' | bash

RUN apt install -y libmc-rtc-dev mc-rtc-utils

RUN apt install -y ros-${ROS_DISTRO}-mc-rtc-plugin ros-${ROS_DISTRO}-mc-rtc-tools

RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
RUN echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
RUN apt-get update && apt-get install -y \
    kitware-archive-keyring \
    cmake \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*~

COPY ./dockerfiles/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /root
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
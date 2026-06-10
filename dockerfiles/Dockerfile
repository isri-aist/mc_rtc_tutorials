# Indicates docker image
FROM osrf/ros:humble-desktop-full

LABEL org.opencontainers.image.source=https://github.com/isri-aist/mc_rtc_tutorials
LABEL org.opencontainers.image.description="mc_rtc_tutorial image"
LABEL org.opencontainers.image.licenses=BSD

# Configure environment
ENV LD_LIBRARY_PATH="/usr/local/lib"

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt install -y curl ca-certificates gpg wget zsh fzf

RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended && \
    git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git clone https://github.com/zsh-users/zsh-completions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-completions
COPY ./dockerfiles/.zshrc /root/.zshrc

RUN curl -1sLf 'https://dl.cloudsmith.io/public/mc-rtc/head/setup.deb.sh' | bash && \
    apt install -y libmc-rtc-dev mc-rtc-utils && \
    apt install -y ros-${ROS_DISTRO}-mc-rtc-plugin ros-${ROS_DISTRO}-mc-rtc-tools && \
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null && \
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null && \
    apt-get update && apt-get install -y kitware-archive-keyring cmake ros-humble-gazebo-ros-pkgs ros-humble-hardware-interface \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies for mc_mujoco
WORKDIR /opt
RUN apt-get update && apt-get install -y libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libglew-dev && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    git clone --recursive https://github.com/rohanpsingh/mc_mujoco.git && \
    cd mc_mujoco && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    make && \
    make install

# Install dependencies for robots_interface
WORKDIR /opt
RUN apt-get update && apt-get install -y software-properties-common \
    && rm -rf /var/lib/apt/lists/* && \
    git clone https://gitlab.com/sdurobotics/ur_rtde.git && \
    cd ur_rtde && \
    mkdir -p build && cd build && \
    cmake -DPYTHON_BINDINGS:BOOL=OFF .. && \
    make && make install

WORKDIR /opt
RUN git clone https://github.com/jrl-umi3218/ur_modern_driver && \
    cd ur_modern_driver && \
    mkdir build && cd build && \
    cmake .. && make install

COPY ./dockerfiles/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
WORKDIR /root
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/zsh"]

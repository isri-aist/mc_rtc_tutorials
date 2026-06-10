#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

echo "--- Building colcon workspace ---"
cd /root/colcon_ws
colcon build
if [[ -f "/root/colcon_ws/install/setup.bash" ]]; then
    source "/root/colcon_ws/install/setup.bash"
fi
echo "--- Colcon workspace build complete ---"

build_project() {
    local name="$1"
    local src="$2"
    if [[ ! -d "$src" ]]; then
        echo "--- Skipping $name (directory not found) ---"
        return 0
    fi
    echo "--- Compiling $name ---"
    mkdir -p "$src/build"
    cd "$src/build"
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
    make -j$(nproc)
    make install
    echo "--- $name compilation complete ---"
}

build_project "mc_robots"    /root/mc_robots
build_project "controllers"  /root/mc_rtc_ws
build_project "mc_mujoco"    /root/mc_mujoco
build_project "mc_interface"  /root/mc_interface

# Source ROS in shell configs
if ! grep -q "^ros/humble/setup.bash" ~/.bashrc 2>/dev/null; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source /root/colcon_ws/install/setup.bash" >> ~/.bashrc
fi
if ! grep -q "^ros/humble/setup.zsh" ~/.zshrc 2>/dev/null; then
    echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
    echo "source /root/colcon_ws/install/setup.zsh" >> ~/.zshrc
fi

if ! grep -q "^RobotModulePaths" /usr/etc/mc_rtc.yaml; then
    echo 'RobotModulePaths: [/usr/lib/mc_robots]' >> /usr/etc/mc_rtc.yaml
fi
if ! grep -q "^ControllerModulePaths" /usr/etc/mc_rtc.yaml; then
    echo 'ControllerModulePaths: [/usr/lib/mc_controller]' >> /usr/etc/mc_rtc.yaml
fi

echo "--- COMPILATION COMPLETED ---"

exec "$@"

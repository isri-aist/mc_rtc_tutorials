#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

cd /root/colcon_ws
colcon build

if [ -f "/root/colcon_ws/install/setup.bash" ]; then
  source "/root/colcon_ws/install/setup.bash"
fi

echo "--- Compiling mc_robots ---"
mkdir -p /root/mc_robots/build
cd /root/mc_robots/build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make install 
echo "--- Compilation Complete ---"


echo "--- Compiling controllers ---"
mkdir -p /root/mc_rtc_ws/build
cd /root/mc_rtc_ws/build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make install 
echo "--- Compilation Complete ---"

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /root/colcon_ws/install/setup.bash" >> ~/.bashrc

exec "$@" 
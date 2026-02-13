# mc_rtc_tutorials

## Introduction
This repository contains various tutorials for [mc_rtc](https://jrl.cnrs.fr/mc_rtc/), an interface for simulation and robot control systems. Each tutorial is organized in its own folder, with code examples and explanations to help users understand different aspects of mc_rtc. Currently, two tutorials are available:
- **dual_arm_controller**: This tutorial demonstrates how to implement a dual-arm controller using C++ and Python. It showcases the control of two robotic arms working together to perform tasks.
- **mobile_arm_controller**: This tutorial focuses on controlling a mobile manipulator robot using C++ and Python. It illustrates how to manage both the mobility and manipulation capabilities of the robot.

## Installation
### With docker (recommended)
To easily set up the environment for running the tutorials, you can use Docker. Follow these steps:
1. Install [Docker Desktop](https://docs.docker.com/get-started/get-docker/) (or [Docker Engine](https://docs.docker.com/engine/install/)) and [Docker Compose](https://docs.docker.com/compose/install/) on your system if you haven't already.
2. Clone this repository to your local machine:
   ```bash
   git clone --recursive https://github.com/HACHIX-CORPORATION/mc_rtc_tutorials.git
   ```
3. Navigate to the `dockerfiles` directory of cloned repository and build, run the Docker container using Docker Compose:
   ```bash
   cd mc_rtc_tutorials/dockerfiles
   docker compose up -d
   ```
4. Access the running container:
   ```bash
   docker exec -it mc_rtc_tutorials bash
   ```
### Without docker
It is not recommended to run the tutorials without Docker since it is mixed between independent CMake projects and ROS2 packages. However, if you want to run the tutorials without Docker, make sure to follow these steps:
1. Install [mc_rtc](https://jrl.cnrs.fr/mc_rtc/) on your system.
2. Clone this repository to your local machine:
   ```bash
   git clone --recursive https://github.com/HACHIX-CORPORATION/mc_rtc_tutorials.git
   ```
3. Build `robots_description` ROS2 packages in a colcon workspace (assuming you have ROS2 Humble installed, and sourced):
   ```zsh
   cd robots_description/
   colcon build
   ```
   Remember to source the workspace after building:
   ```bash
   source install/setup.bash
   ```
4. Navigate to the tutorial directories and build them using CMake for C++ examples or run the Python scripts directly.
    ```
    mkdir build && cd build
    cmake ..
    make && make install
    ```

## Running preinstalled controller
### dual_arm_controller C++ tutorial
To run the dual_arm_controller tutorial, in one terminal, run the mc_rtc_ticker:
```bash
mc_rtc_ticker -f config/dual_arm_controller.yaml
```
In another terminal, run RVIZ2 for visualization:
```bash
ros2 launch mc_rtc_ticker display.launch
```
### dual_arm_controller Python tutorial
To run the dual_arm_controller Python tutorial, in one terminal, run the mc_rtc_ticker:
```bash
PYTHONPATH=<path_to_mc_rtc_tutorials>/dual_arm_controller/python mc_rtc_ticker -f config/dual_arm_controller_python.yaml
```
In another terminal, run RVIZ2 for visualization:
```bash
ros2 launch mc_rtc_ticker display.launch
```
### mobile_arm_controller C++ tutorial
To run the mobile_arm_controller tutorial, in one terminal, run the mc_rtc_ticker:
```bash
mc_rtc_ticker -f config/mobile_arm_controller.yaml
```
In another terminal, run RVIZ2 for visualization:
```bash
ros2 launch mc_rtc_ticker display.launch
```
### mobile_arm_controller Python tutorial
To run the mobile_arm_controller Python tutorial, in one terminal, run the mc_rtc_ticker:
```bash
PYTHONPATH=<path_to_mc_rtc_tutorials>/mobile_arm_controller/python mc_rtc_ticker -f config/mobile_arm_controller_python.yaml
```
In another terminal, run RVIZ2 for visualization:
```bash
ros2 launch mc_rtc_ticker display.launch
```

## Adding controller
You can add controllers of your own and build it easily.

Put your new controller inside `controllers` directory. If you have never work with `mc_rtc` before, you can find several tutorials to get started [here](https://jrl.cnrs.fr/mc_rtc/tutorials.html).
```bash
cd controllers
```
List your new controller in `~/controllers/CMakeLists.txt`.

You can exit the container and recompose it.
```bash
docker compose up -d
docker exec -it mc_rtc_tutorials bash
```

If you are not using Docker or do not want to exit your container. Navigate to the `build` directory and rebuild the project.
```bash
cd build/
cmake ..
make
sudo make install
```
# mc_rtc_tutorials

## Introduction
This repository contains various tutorials for [mc_rtc](https://jrl.cnrs.fr/mc_rtc/), an interface for simulation and robot control systems. Each tutorial is organized in its own folder, with code examples and explanations to help users understand different aspects of mc_rtc. Currently, two tutorials are available:
- **dual_arm_controller**: This tutorial demonstrates how to implement a dual-arm controller using C++ and Python. It showcases the control of two robotic arms working together to perform tasks.
- **mobile_arm_controller**: This tutorial focuses on controlling a mobile manipulator robot using C++ and Python. It illustrates how to manage both the mobility and manipulation capabilities of the robot.

Mujoco description for UR5e and Kinova robots are also provided to run controller with [mc_mujoco](https://github.com/rohanpsingh/mc_mujoco), a MuJoCo interface for mc_rtc.

## Installation
Clone this repository to your local machine:
```bash
git clone --recursive https://github.com/isri-aist/mc_rtc_tutorials.git
```
### With docker (recommended)
To easily set up the environment for running the tutorials, you can use Docker. Follow these steps:
1. Install [Docker Engine](https://docs.docker.com/engine/install/) and [Docker Compose](https://docs.docker.com/compose/install/) on your system if you haven't already.
2. Navigate to the `dockerfiles` directory of cloned repository and build, run the Docker container using Docker Compose:
   ```bash
   cd mc_rtc_tutorials/dockerfiles
   docker compose up -d
   ```
3. Access the running container:
   ```bash
   docker exec -it mc_rtc_tutorials bash
   ```
### Without docker
It is not recommended to run the tutorials without Docker since it is mixed between independent CMake projects and ROS2 packages. However, if you want to run the tutorials without Docker, make sure to follow these steps:
1. Install [mc_rtc](https://jrl.cnrs.fr/mc_rtc/) on your system.
2. Build `robots_description` ROS2 packages in a colcon workspace (assuming you have ROS2 Humble installed, and sourced):
   ```zsh
   cd robots_description/
   colcon build
   ```
   Remember to source the workspace after building:
   ```bash
   source install/setup.bash
   ```
3. Navigate to the tutorial directories and build them using CMake for C++ examples or run the Python scripts directly.
    ```
    mkdir build && cd build
    cmake ..
    make && make install
    ```

## mc_rtc controller

In one terminal, run RVIZ2 for visualization:
```bash
ros2 launch mc_rtc_ticker display.launch
```

In a separate terminal, use mc_rtc_ticker to run the tutorials.
### dual_arm_controller
#### C++
```bash
mc_rtc_ticker -f ~/config/dual_arm_controller.yaml
```
#### Python
```bash
PYTHONPATH=<path_to_mc_rtc_tutorials>/controllers/dual_arm_controller/python mc_rtc_ticker -f ~/config/dual_arm_controller_python.yaml
```
If you are using docker: `PYTHONPATH=${HOME}/mc_rtc_ws/dual_arm_controller/python mc_rtc_ticker -f ~/config/dual_arm_controller_python.yaml`

### mobile_arm_controller
#### C++
```bash
mc_rtc_ticker -f ~/config/mobile_arm_controller.yaml
```
#### Python
```bash
PYTHONPATH=<path_to_mc_rtc_tutorials>/controllers/mobile_arm_controller/python mc_rtc_ticker -f ~/config/mobile_arm_controller_python.yaml
```
If you are using docker: `PYTHONPATH=${HOME}/mc_rtc_ws/mobile_arm_controller/python mc_rtc_ticker -f ~/config/mobile_arm_controller_python.yaml`

### Adding controller
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
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build --parallel -j$(nproc)
cmake --install build
```

## mc_mujoco

With the provided mc_mujoco descriptions, you can run `dual_arm_controller` with mc_mujoco
```sh
mc_mujoco -sf ~/config/dual_arm_controller.yaml
```

You can add other controllers for UR5e and Kinova and simulate them with mc_mujoco using the same command.

```sh
mc_mujoco -sf <path_to_config_file>
```

## mc_rtc interface

The project also provides a mc_rtc robot interface, [mc_rtde](https://github.com/isri-aist/mc_rtde). To run this, you can use the config file `rtde.yaml`

```sh
MCControlRtde -f ~/config/rtde.yaml
```


## Note

For both `mc_rtc_ticker`, `mc_mujoco`, and `MCControlRtde`, you can run them with flag `-h` or `--help` to see all available options.

This tutorial includes many submodules which are updated frequently. If you have any building issues. Consider update github submodules.

```sh
git submodule update --remote --recursive
```

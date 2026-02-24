cmake_minimum_required(VERSION 3.22)

project(mc_rtc_tutorial_robots_interface)

set(PYTHON_BINDINGS OFF)
add_subdirectory(ur_rtde)
unset(PYTHON_BINDINGS)
add_subdirectory(mr_rtde)

# list new controller here
# add_subdirectory(<your_controller>)
# list new interface here
# add_subdirecotry(<your_controller>)
# !!! dependencies has to be added before interfaces !!!


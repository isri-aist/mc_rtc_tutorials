cmake_minimum_required(VERSION 3.21)
cmake_policy(SET CMP0048 NEW)

option(DISABLE_ROS "Compile all dependencies in ROS-free environment" OFF)
set(PROJECT_VERSION 0.0.0)
project(mc_rtc_tutorial VERSION ${PROJECT_VERSION})

enable_testing()

if(DISABLE_ROS)
  message(STATUS "Building without ROS support as DISABLE_ROS=ON")
else()
  if(NOT DEFINED ENV{ROS_VERSION})
    message(FATAL_ERROR "ROS_VERSION is not set in the environment")
  endif()
  set(ROS_VERSION $ENV{ROS_VERSION})
endif()

# Automatically add all installed robot descriptions
set(DESCRIPTIONS_INSTALL_DIR "${CMAKE_CURRENT_LIST_DIR}/robots_description/install")
if(EXISTS "${DESCRIPTIONS_INSTALL_DIR}")
  list(APPEND CMAKE_PREFIX_PATH "${DESCRIPTIONS_INSTALL_DIR}")
  message(STATUS "Added robot descriptions to CMAKE_PREFIX_PATH: ${DESCRIPTIONS_INSTALL_DIR}")
endif()


add_subdirectory(controllers)
add_subdirectory(robots)
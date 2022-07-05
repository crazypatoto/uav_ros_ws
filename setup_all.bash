#!/bin/bash

PX4_AUTOPILOT_DIR=../PX4-Autopilot
SRC_DIR=${PX4_AUTOPILOT_DIR}
BUILD_DIR=${PX4_AUTOPILOT_DIR}/build/px4_sitl_default

### Check if a directory does not exist ###
if [ ! -d $PX4_AUTOPILOT_DIR ] 
then
    echo "Directory $PX4_AUTOPILOT_DIR dose not exists." 
    echo "Please set correct PX4_AUTOPILOT_DIR."
    return 1
fi


source ./devel/setup.bash 

# setup Gazebo env and update package path
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${BUILD_DIR}/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${SRC_DIR}/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${BUILD_DIR}/build_gazebo

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PX4_AUTOPILOT_DIR}
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PX4_AUTOPILOT_DIR}/Tools/sitl_gazebo

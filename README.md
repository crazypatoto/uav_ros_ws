# UAV Simulation ROS Worksapce

## Start the simulation

1. Go into PX4-Autopilot directory:
```bash
cd $PX4_Autopilot_Directory
```
2. Source the evnironment:
```bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
```
3. Run the launch file:
```bash
roslaunch px4 custom_single.launch
```
4. Go into current workspace:
```bash
cd $ROS_WORKSPACE_DIRECTORY
```
5. Setup ROS environment
```bash
source devel/setup.bash
```

6. Run offboard control node:
```bash
roslaunch move_uav single_uav.launch 
```

7. Run keyboard control node:
```bash
rosrun offboard_pkg keyboard_move_node 
```

## Alternative Method
1. Go into PX4-Autopilot directory:
```bash
source ~/uav_ros_ws/devel/setup.bash 
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```
2. Run
```bash
roslaunch move_uav single_uav.launch 
```
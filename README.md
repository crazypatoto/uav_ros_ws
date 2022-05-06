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

5. Run offboard control node:
```bash
rosrun offboard_pkg offboard_node
```

6. Run keyboard control node:
```bash
rosrun offboard_pkg keyboard_move_node 
```
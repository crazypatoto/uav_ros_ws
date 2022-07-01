# UAV Simulation ROS Worksapce

## Packages
- ### *[gazebo_ros_link_attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher)  
Attach two Gazebo models with a virtual joint in a generalized grasp hack. 
- ### *[mavros_controller](https://github.com/Jaeyoung-Lim/mavros_controllers)
Controllers for controlling MAVs using the mavros package in OFFBOARD mode.  
    * geometric_controller: Trajectory tracking controller based on geometric control
- ### move_uav
- ### offboard_pkg
- ### ruckig_ros
- ### uav_controller
- ### uav_gazebo
- ### uav_launch
- ### uav_mission_handler
- ### uav_msgs


## Prerequisites
1. Install Dependencies
```bash
sudo apt-get install libeigen3-dev
```

2. Install PX4 Autopilot Firmare (v1.12.3)
```bash
git clone -b v1.12.3 https://github.com/crazypatoto/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
git checkout stable
make px4_sitl gazebo
```
2. Copy Modified iris model into PX4 sitl_gazebo model path
```bash
cp ~/uav_ros_ws/src/uav_gazebo/models/iris_with_standoffs/iris.sdf.jinja  ~/PX4-Autopilot/Tools/sitl_gazebo/models/iris/iris.sdf.jinja
```

3. Setup PX4_AUTOPILOT_DIR in setup_all.bash
```bash
gedit setup_all.bash
```

## Start the simulation

1. Source the environment:
```bash
source setup_all.bash
```

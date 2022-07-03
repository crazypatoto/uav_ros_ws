# UAV Simulation ROS Worksapce

## Packages
- ### *[gazebo_ros_link_attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher)*
    Attach two Gazebo models with a virtual joint in a generalized grasp hack. 
- ### **[mavros_controller](https://github.com/Jaeyoung-Lim/mavros_controllers)*
    Controllers for controlling MAVs using the mavros package in OFFBOARD mode.  
    * geometric_controller: Trajectory tracking controller based on geometric control
    * controller_msgs: custom message definitions
    * trajectory_publisher: Node publishing setpoints as states from motion primitives / trajectories for the controller to follow
- ### mission_dispatcher
    Mission scheduler and dispatcher.
- ### *move_uav
    Simple uav control in OFFBOARD mode.
- ### *offboard_pkg
    An example package according to [offical PX4 OFFBOARD mode turtorial](https://docs.px4.io/v1.12/en/ros/mavros_offboard.html).
- ### *ruckig_ros*
    Custom made ros package version of ruckig, an on-the-fly trajectory generation library. Modified from [ruckig](https://github.com/pantor/ruckig) by [pantor](https://github.com/pantor).
- ### uav_controller
    Core UAV controlling package. Including attitude control, trajectory generation and tracking, taking off and landing.
- ### uav_gazebo
    Including modified iris models and test environments.
- ### uav_launch
    Including uav launch files and rviz configs.
- ### uav_mission_handler
    UAV mission handler.
- ### uav_msgs
    Custom UAV messages.

*Note: Package names with * prefix are not necessary.*


## Prerequisites
1. Install Dependencies
```bash
sudo apt-get install libeigen3-dev
sudo apt-get install libtinyxml-dev
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

# UAV Simulation ROS Worksapce

## Prerequisites
1. Install PX4 Autopilot Firmare (v1.12.3)
```bash
git clone -b v1.12.3 https://github.com/crazypatoto/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
git checkout stable
make px4_sitl gazebo
```
2. Install Dependencies
```bash
sudo apt-get install libeigen3-dev
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

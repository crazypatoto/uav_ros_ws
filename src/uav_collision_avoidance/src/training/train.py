#!/usr/bin/env python3
from datetime import date
from os import stat
import subprocess
from subprocess import Popen
import time
from tkinter import Y
from urllib import response
import math

import rospy
import roslaunch
import rospkg
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from mavros_msgs.msg import State as MavState
from uav_msgs.srv import Land
from uav_msgs.msg import State as UAVState

SIM_SPEED_FACTOR = 5.0
X_SIZE = 100
Y_SIZE = 100


class train():
    def __init__(self):

        self.env = SimulationEnvironment()
        self.env.set_speed_factor(SIM_SPEED_FACTOR)
        self.env.start_controllers()

        i = 0
        while True:
            while self.env.uav0_state + self.env.uav1_state + self.env.uav2_state + self.env.uav3_state < 16:
                rospy.sleep(1)            
            rospy.loginfo('Exiting at i = %d', i)
            self.env.land_all_uav()
            self.env.wait_uav_disarmed()
            self.env.stop_controllers()
            self.env.set_uav_pose(0, 0.0, 0.0, 0.0, 1.0)
            self.env.set_uav_pose(1, 1.0, 0.0, 0.0, 2.0)
            self.env.set_uav_pose(2, 2.0, 0.0, 0.0, 3.0)
            self.env.set_uav_pose(3, 3.0, 0.0, 0.0, 4.0)
            self.env.start_controllers()
            i = i+1


class SimulationEnvironment():
    def __init__(self):
        rospy.Subscriber("/uav0/uav_controller/state",
                         UAVState, self.uav0_state_callback)
        rospy.Subscriber("/uav1/uav_controller/state",
                         UAVState, self.uav1_state_callback)
        rospy.Subscriber("/uav2/uav_controller/state",
                         UAVState, self.uav2_state_callback)
        rospy.Subscriber("/uav3/uav_controller/state",
                         UAVState, self.uav3_state_callback)

        rospy.Subscriber("/uav0/mavros/state",
                         MavState, self.mav0_state_callback)
        rospy.Subscriber("/uav1/mavros/state",
                         MavState, self.mav1_state_callback)
        rospy.Subscriber("/uav2/mavros/state",
                         MavState, self.mav2_state_callback)
        rospy.Subscriber("/uav3/mavros/state",
                         MavState, self.mav3_state_callback)

        self.set_model_state = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)

        self.uav0_state = -1
        self.uav1_state = -1
        self.uav2_state = -1
        self.uav3_state = -1

    def uav0_state_callback(self, data):
        self.uav0_state = data.state

    def uav1_state_callback(self, data):
        self.uav1_state = data.state

    def uav2_state_callback(self, data):
        self.uav2_state = data.state

    def uav3_state_callback(self, data):
        self.uav3_state = data.state

    def mav0_state_callback(self, data):
        self.uav0_armed = data.armed

    def mav1_state_callback(self, data):
        self.uav1_armed = data.armed

    def mav2_state_callback(self, data):
        self.uav2_armed = data.armed

    def mav3_state_callback(self, data):
        self.uav3_armed = data.armed

    def start_controllers(self):
        r = rospkg.RosPack()
        gazobo_pkg_path = r.get_path('uav_collision_avoidance')
        cli_args = [gazobo_pkg_path +
                    "/launch/start_controllers.launch", '']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [
            (roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(
            uuid, roslaunch_file)
        self.launch.start()

    def stop_controllers(self):
        self.launch.shutdown()
        self.uav0_state = -1
        self.uav1_state = -1
        self.uav2_state = -1
        self.uav3_state = -1

    def set_speed_factor(self, factor):
        rospy.wait_for_service('/gazebo/get_physics_properties')
        try:
            get_physics_properties = rospy.ServiceProxy(
                '/gazebo/get_physics_properties', GetPhysicsProperties)
            resp = get_physics_properties()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        rospy.wait_for_service('/gazebo/set_physics_properties')
        try:
            set_physics_properties = rospy.ServiceProxy(
                '/gazebo/set_physics_properties', SetPhysicsProperties)
            resp = set_physics_properties(0.004, 250 * factor,
                                          resp.gravity, resp.ode_config)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        if resp.success == False:
            rospy.logerr('Cannot set speed factor')

    def wait_uav_disarmed(self):
        while (self.uav0_armed or self.uav1_armed or self.uav2_armed or self.uav3_armed):
            rospy.sleep(0.1)

    def set_uav_pose(self, uav_id, x, y, z, yaw):
        state = ModelState()
        state.model_name = "iris" + str(uav_id)
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.w = math.sin(yaw / 2)
        state.pose.orientation.z = math.cos(yaw / 2)

        self.set_model_state(state)

    def land_all_uav(self):
        rospy.wait_for_service('/uav0/uav_controller/land')
        rospy.wait_for_service('/uav1/uav_controller/land')
        rospy.wait_for_service('/uav2/uav_controller/land')
        rospy.wait_for_service('/uav3/uav_controller/land')
        try:
            land_uav0 = rospy.ServiceProxy(
                '/uav0/uav_controller/land', Land)     
            land_uav1 = rospy.ServiceProxy(
                '/uav1/uav_controller/land', Land)
            land_uav2 = rospy.ServiceProxy(
                '/uav2/uav_controller/land', Land)
            land_uav3 = rospy.ServiceProxy(
                '/uav3/uav_controller/land', Land)    
            land_uav0()
            land_uav1()
            land_uav2()
            land_uav3()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


def main():

    # Initialize the node and name it.
    rospy.init_node('uav_collision_avoidance_training_node',)
    rospy.loginfo('Start UAV collision training...')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        train()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import os
import time
from urllib import response

import rospy
import roslaunch
import rospkg
import rosservice
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties

SIM_SPEED_FACTOR = 5.0


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


class train():
    def __init__(self):
        self.env = SimulationEnvironment()
        # for i in range(100):
        #     self.env.start()
        #     time.sleep(50)
        #     self.env.stop()
        #     time.sleep(3)
        #     print(i)

        self.env.start()
        time.sleep(10)
        self.env.stop()
        time.sleep(3)


class SimulationEnvironment():

    def __init__(self):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        roslaunch.logging.disable()
        r = rospkg.RosPack()
        gazobo_pkg_path = r.get_path('uav_collision_avoidance')
        cli_args = [gazobo_pkg_path + "/launch/train.launch", '']
        roslaunch_args = cli_args[1:]
        self.roslaunch_file = [
            (roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

        pass

    def start(self):
        # os.environ['PX4_SIM_SPEED_FACTOR'] = str(SIM_SPEED_FACTOR)

        self.launch = roslaunch.parent.ROSLaunchParent(
            self.uuid, self.roslaunch_file)
        self.launch.start()

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
            set_physics_properties(0.004, 1000, resp.gravity, resp.ode_config)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def stop(self):
        os.system('pkill gzclient')
        os.system('pkill gzserver')
        self.launch.shutdown()


if __name__ == '__main__':
    main()

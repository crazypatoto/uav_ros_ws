#!/usr/bin/env python3
import rospy


class UAVCollisionAvoidanceNode():
    def __init__(self):
        pass
    
if __name__ == '__main__':    
    # Initialize the node and name it.
    rospy.init_node('uav_collision_avoidance_node')
    rospy.loginfo('UAV Collision Node Start...')

    # Go to class functions that do all the heavy lifting. Do error checking.    
    try:          
        node = UAVCollisionAvoidanceNode()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

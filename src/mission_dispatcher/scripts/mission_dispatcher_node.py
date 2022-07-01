#!/usr/bin/env python3
import rospy


class MissionDispatcherNode():
    def __init__(self):
        pass
    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('mission_dispatcher_node')

    # Go to class functions that do all the heavy lifting. Do error checking.    
    try:          
        node = MissionDispatcherNode()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

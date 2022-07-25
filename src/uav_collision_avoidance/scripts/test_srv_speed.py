#!/usr/bin/env python
import time
import rospy
from geometry_msgs.msg import Vector3
from uav_msgs.srv import UCASelectAction, UCASelectActionRequest
from uav_msgs.srv import UAVsInRange, UAVsInRangeRequest

class Test():
    def __init__(self):
        rospy.wait_for_service('uav_collision_avoidance/select_action')
        rospy.wait_for_service('uav_collision_avoidance/uavs_in_range')
        rospy.loginfo('Service ready!')
        test_uav = 'uav3'
        
        t = time.time()
        for i in range(1000):
            try:
                select_action = rospy.ServiceProxy('uav_collision_avoidance/select_action', UCASelectAction)
                req = UCASelectActionRequest()
                req.uavNamespace = test_uav
                req.target = Vector3(0, 0, 0)
                resp = select_action(req)                
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        timespan = time.time() - t
        print(timespan)
        print(timespan / 1000)

        t = time.time()
        for i in range(1000):
            try:
                select_action = rospy.ServiceProxy('uav_collision_avoidance/uavs_in_range', UAVsInRange)
                req = UAVsInRangeRequest()
                req.uavNamespace = test_uav               
                resp = select_action(req)                
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        timespan = time.time() - t
        print(timespan)
        print(timespan / 1000)



if __name__ == '__main__':    
    # Initialize the node and name it.
    rospy.init_node('test_srv_speed_node')    

    # Go to class functions that do all the heavy lifting. Do error checking.    
    try:          
        node = Test()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

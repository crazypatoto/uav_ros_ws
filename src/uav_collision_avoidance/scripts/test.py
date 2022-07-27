#!/usr/bin/env python
import time
import rospy
from geometry_msgs.msg import Vector3
from uav_msgs.srv import UCASelectAction, UCASelectActionRequest
from uav_msgs.srv import UAVsInRange, UAVsInRangeRequest
from std_srvs.srv import Trigger, TriggerResponse

class Test():
    def __init__(self):
        rospy.Service('/service1', Trigger, self.handle_service1)
        rospy.Service('/service2', Trigger, self.handle_service2)
        pass
    
    def handle_service1(self, req):
        response = TriggerResponse()
        response.success = True        
        time.sleep(10)
        seconds = rospy.get_time()
        response.message = "Service 1 @ %.3f" % seconds
        return response
        

    def handle_service2(self, req):
        response = TriggerResponse()
        response.success = True        
        rospy.sleep(1)
        seconds = rospy.get_time()
        response.message = "Service 2 @ %.3f" % seconds
        return response
        

if __name__ == '__main__':    
    # Initialize the node and name it.
    rospy.init_node('test_srv_speed_node')    

    # Go to class functions that do all the heavy lifting. Do error checking.    
    try:          
        node = Test()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

#!/usr/bin/env python
from concurrent.futures import thread
import time
import threading
import rospy
from geometry_msgs.msg import Vector3
from uav_msgs.srv import UCASelectAction, UCASelectActionRequest
from uav_msgs.srv import UAVsInRange, UAVsInRangeRequest

class Test():
    def __init__(self):        
        # rospy.wait_for_service('uav_collision_avoidance/uavs_in_range')
        rospy.loginfo('Service ready!')
        t0 = threading.Thread(target=self.test_srv, args=('server1', 'uav0', 100))
        t1 = threading.Thread(target=self.test_srv, args=('server2', 'uav1', 100))
        t2 = threading.Thread(target=self.test_srv, args=('server3', 'uav2', 100))
        t3 = threading.Thread(target=self.test_srv, args=('server4', 'uav3', 100))
        t4 = threading.Thread(target=self.test_srv, args=('server5', 'uav4', 100))
        t5 = threading.Thread(target=self.test_srv, args=('server1', 'uav5', 100))
        t6 = threading.Thread(target=self.test_srv, args=('server2', 'uav6', 100))
        t7 = threading.Thread(target=self.test_srv, args=('server3', 'uav7', 100))
        t8 = threading.Thread(target=self.test_srv, args=('server4', 'uav8', 100))
        t9 = threading.Thread(target=self.test_srv, args=('server5', 'uav9', 100))
        t0.start()
        t1.start()
        t2.start()
        t3.start()
        t4.start()
        t5.start()
        t6.start()
        t7.start()
        t8.start()
        t9.start()
                
    def test_srv(self, server_name_space, uav_namespace, num):
        t = time.time()
        for _ in range(num):
            try:
                select_action = rospy.ServiceProxy('/{}/uav_collision_avoidance/select_action'.format(server_name_space), UCASelectAction)
                # select_action = rospy.ServiceProxy('/uav_collision_avoidance/select_action'.format(uav_namespace), UCASelectAction)
                req = UCASelectActionRequest()
                req.uavNamespace = uav_namespace
                req.target = Vector3(0, 0, 0)
                resp = select_action(req)                
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        timespan = time.time() - t
        print('Average service call time of %s = %.5f secs' % (uav_namespace, timespan/num))                



if __name__ == '__main__':    
    # Initialize the node and name it.
    rospy.init_node('test_srv_speed_node')    

    # Go to class functions that do all the heavy lifting. Do error checking.    
    try:          
        node = Test()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

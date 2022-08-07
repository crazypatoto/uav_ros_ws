#!/usr/bin/env python
from curses import REPORT_MOUSE_POSITION
from locale import normalize
from tkinter.messagebox import NO
from unicodedata import name
from urllib import response
import rospy
import rospkg
import os
import math
import torch
import numpy as np
import std_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from uav_msgs.srv import UCASelectAction, UCASelectActionResponse
from uav_msgs.srv import UAVsInRange, UAVsInRangeResponse
from pytorch_sac.sac import SAC

UAV_COLLISION_ALTITUDE_THRESHOLD = 3
UAV_SENSE_RANGE = 15
UAV_SENSET_TRIGGER_RANGE = 30
DISTANCE_FACTOR = 50 / math.cos(math.pi/4)
MAX_SPEED = 10
MAX_ACCELERATION = 5

class UAVCollisionAvoidanceServerNode():
    def __init__(self):             
        self.uav_namespaces = self.get_all_uav_namespaces()
        self.sub_to_all_uavs(self.uav_namespaces)         
        rospy.Service('uav_collision_avoidance/select_action', UCASelectAction, self.handle_action_selection)
        rospy.Service('uav_collision_avoidance/uavs_in_range', UAVsInRange, self.handle_uavs_in_range)
        use_cpu = rospy.get_param('use_cpu', False)
        self.agent = SAC(10, 2, use_cpu=use_cpu) # 10 observations, 2 actions
        rospack = rospkg.RosPack()                
        path = rospack.get_path('uav_collision_avoidance')        
        path = os.path.join(path, 'weights')        
        self.agent.load_checkpoint(path, evaluate=True)
     
    def handle_action_selection(self, req):
        ns = req.uavNamespace
        target = req.target
        target = np.array([target.x, target.y])

        response = UCASelectActionResponse()
        if ns not in self.uav_namespaces:
            response.success = False            
        else:            
            observation = self.get_observation(ns, target)                
            action = self.agent.select_action(observation, evaluate=True)            
            v = (action[0]/2+0.5) * MAX_SPEED
            theta = action[1] * math.pi
            converted_action = Vector3()
            converted_action.x = v*math.cos(theta)
            converted_action.y = v*math.sin(theta)
            response.action = converted_action
            response.success = True
            # print("%s: %.2f, %.2f" % (ns, converted_action.x, converted_action.y))
        
        return response

    def handle_uavs_in_range(self, req):
        ns = req.uavNamespace        

        response = UAVsInRangeResponse()
        if ns not in self.uav_namespaces:
            response.success = False            
        else:                
            current_uav = self.uavs[ns]
            uavs_in_range = current_uav.uavs_in_range(list(self.uavs.values()), UAV_SENSET_TRIGGER_RANGE)
            uav_key_list = list(self.uavs.keys())
            uav_value_list = list(self.uavs.values())
            uav_namespace_list = []
            for uav in uavs_in_range:
                uav_namespace_list.append(uav_key_list[uav_value_list.index(uav)])
            response.uavs = uav_namespace_list
            response.success = True
        
        return response

    def get_all_uav_namespaces(self):
        uav_namespaces = []
        topics = rospy.get_published_topics()
        for topic in topics:
            topic = topic[0].split('/')[1]
            if 'uav' in topic:
                if topic not in uav_namespaces:
                    uav_namespaces.append(topic)
       
        return uav_namespaces
    
    def sub_to_all_uavs(self, namespaces):
        self.uavs = {}
        for ns in namespaces:
            self.uavs[ns] = UAV()
            rospy.Subscriber('/{}/ground_truth/state'.format(ns), Odometry, self.uav_pose_callbacks, ns)
    
    def uav_pose_callbacks(self, data, namespace):        
        self.uavs[namespace].position[0] = data.pose.pose.position.x
        self.uavs[namespace].position[1] = data.pose.pose.position.y
        self.uavs[namespace].velocity[0] = data.twist.twist.linear.x
        self.uavs[namespace].velocity[1] = data.twist.twist.linear.y
        self.uavs[namespace].altitude = data.pose.pose.position.z    

    def get_observation(self, uav_namespace, target):
        current_agent = self.uavs[uav_namespace]
        normalized_agent_speed = np.linalg.norm(current_agent.velocity) / MAX_SPEED
        agent_theta = math.atan2(current_agent.velocity[1], current_agent.velocity[0])
        normalized_agent_theta = agent_theta / math.pi
        normalized_target_distance = np.linalg.norm(target - current_agent.position) / DISTANCE_FACTOR
        normalized_target_distance = min(normalized_target_distance, 1)        
        target_theta = math.atan2((target - current_agent.position)[1], (target - current_agent.position)[0])
        delta_theta = target_theta - agent_theta
        delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))
        normalized_delta_theta = delta_theta / math.pi

        # Obstacle States
        obstacles = current_agent.uavs_in_range(list(self.uavs.values()), UAV_SENSE_RANGE)
        
        normalized_obs1_relative_distance = (np.linalg.norm(obstacles[0].position - current_agent.position) / UAV_SENSE_RANGE) if len(obstacles) > 0 else 1
        relative_obs1_theta = (math.atan2((obstacles[0].position - current_agent.position)[1],(obstacles[0].position - current_agent.position)[0])) if len(obstacles) > 0 else (math.pi + agent_theta)
        delta_theta_obs1 = relative_obs1_theta - agent_theta
        delta_theta_obs1 = math.atan2(math.sin(delta_theta_obs1), math.cos(delta_theta_obs1))
        normalized_delta_theta_obs1 = delta_theta_obs1 / math.pi
        obs1_direction = (math.atan2(obstacles[0].position[1],obstacles[0].position[0])) if len(obstacles) > 0 else (agent_theta)
        relative_obs1_direction = obs1_direction - agent_theta
        relative_obs1_direction = math.atan2(math.sin(relative_obs1_direction), math.cos(relative_obs1_direction))
        normalized_relative_obs1_direction = relative_obs1_direction / math.pi

        normalized_obs2_relative_distance = (np.linalg.norm(obstacles[1].position - current_agent.position) / UAV_SENSE_RANGE) if len(obstacles) > 1 else 1
        relative_obs2_theta = (math.atan2((obstacles[1].position - current_agent.position)[1],(obstacles[1].position - current_agent.position)[0])) if len(obstacles) > 1 else (math.pi + agent_theta)
        delta_theta_obs2 = relative_obs2_theta - agent_theta
        delta_theta_obs2 = math.atan2(math.sin(delta_theta_obs2), math.cos(delta_theta_obs2))
        normalized_delta_theta_obs2 = delta_theta_obs2 / math.pi
        obs2_direction = (math.atan2(obstacles[1].velocity[1],obstacles[1].velocity[0])) if len(obstacles) > 1 else (agent_theta)
        relative_obs2_direction = obs2_direction - agent_theta
        relative_obs2_direction = math.atan2(math.sin(relative_obs2_direction), math.cos(relative_obs2_direction))
        normalized_relative_obs2_direction = relative_obs2_direction / math.pi

        return np.array([
                            normalized_agent_speed,
                            normalized_agent_theta,
                            normalized_target_distance,
                            normalized_delta_theta,
                            normalized_obs1_relative_distance,
                            normalized_delta_theta_obs1,
                            normalized_relative_obs1_direction,
                            normalized_obs2_relative_distance,
                            normalized_delta_theta_obs2,
                            normalized_relative_obs2_direction,                            
                        ])

class UAV():
    def __init__(self):        
        self.position = np.zeros(2)
        self.velocity = np.zeros(2)
        self.altitude = 0

    def uavs_in_range(self, uav_list, sense_range):       
        uavs = []
        relative_distances = []
        for i in range(len(uav_list)):
            target_agent = uav_list[i]
            if target_agent == self:
                continue
            distance = np.linalg.norm(target_agent.position - self.position)
            altitude_difference = abs(target_agent.altitude - self.altitude)
            if distance < sense_range and altitude_difference < UAV_COLLISION_ALTITUDE_THRESHOLD:
                uavs.append(target_agent)
                relative_distances.append(distance)
        
        # Sort UAVs with relative distances 
        if len(uavs) == 0:
            return []
        else:
            uavs = np.array(uavs)
            relative_distances = np.array(relative_distances)
            inds = relative_distances.argsort()
            sorted_uavs = uavs[inds]
        return sorted_uavs.tolist()
    
if __name__ == '__main__':    
    # Initialize the node and name it.
    rospy.init_node('uav_collision_avoidance_server_node')
    rospy.loginfo('UAV Collision Server Node Start...')
    rospy.loginfo('Using {}'.format(torch.device("cuda" if torch.cuda.is_available() else "cpu")) )    

    # Go to class functions that do all the heavy lifting. Do error checking.    
    try:          
        node = UAVCollisionAvoidanceServerNode()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()


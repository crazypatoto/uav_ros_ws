#!/usr/bin/env python
from locale import normalize
from tkinter.messagebox import NO
from unicodedata import name
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
from pytorch_sac.sac import SAC


UAV_SENSE_RANGE = 30
DISTANCE_FACTOR = 100 / math.cos(math.pi/4)
MAX_SPEED = 10 #12 / math.cos(math.pi/4)
MAX_ACCELERATION = 5
UPDATE_RATE = 50


class UAVCollisionAvoidanceNode():
    def __init__(self, current_uav_namespace):
        self.namespace = current_uav_namespace        
        self.uav_namespaces = self.get_all_uav_namespaces()
        self.sub_to_all_uavs(self.uav_namespaces)
        self.target = None
        self.prev_velocity = np.zeros(2)        
        self.cmd_vel_pub = rospy.Publisher('uav_controller/cmd_vel', TwistStamped, queue_size=1)
        rospy.Subscriber('uav_collision_avoidance/target', Vector3, self.temp_target_callback)        
        agent = SAC(10, 2) # 10 observations, 2 actions
        rospack = rospkg.RosPack()                
        path = rospack.get_path('uav_collision_avoidance')        
        path = os.path.join(path, 'weights')        
        agent.load_checkpoint(path, evaluate=True)

        rate = rospy.Rate(UPDATE_RATE) # 50Hz
        while True:
            if self.target is not None:
                observation = self.get_observation()
                # print(observation)
                action = agent.select_action(observation, evaluate=True)                
                self.public_action(action)
            rate.sleep()
                        

    def temp_target_callback(self, data):
        self.target = np.array([data.x, data.y])
        self.prev_velocity = self.uavs[self.namespace].velocity

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

    def public_action(self, action):
        v = (action[0]/2+0.5) * MAX_SPEED
        theta = action[1] * math.pi        
        vx = v*math.cos(theta)
        vy = v*math.sin(theta)     
        target_velocity = np.array([vx ,vy])

        current_agent = self.uavs[self.namespace]
        max_acceleration = np.array([MAX_ACCELERATION, MAX_ACCELERATION])
        max_speed = np.array([MAX_SPEED, MAX_SPEED])
        dt = (1/UPDATE_RATE)
        dv = np.clip((target_velocity - self.prev_velocity)/dt, -max_acceleration, max_acceleration)
        target_velocity = np.clip(self.prev_velocity + dv * dt, -max_speed, max_speed)
        self.prev_velocity = target_velocity

        msg = TwistStamped()
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        msg.header = h
        msg.twist.linear.x = target_velocity[0]
        msg.twist.linear.y = target_velocity[1]
        msg.twist.linear.z = 0

        self.cmd_vel_pub.publish(msg)

    def get_observation(self):
        current_agent = self.uavs[self.namespace]
        normalized_agent_speed = np.linalg.norm(current_agent.velocity) / MAX_SPEED
        agent_theta = math.atan2(current_agent.velocity[1], current_agent.velocity[0])
        normalized_agent_theta = agent_theta / math.pi
        normalized_target_distance = np.linalg.norm(self.target - current_agent.position) / DISTANCE_FACTOR
        normalized_target_distance = min(normalized_target_distance, 1)        
        target_theta = math.atan2((self.target - current_agent.position)[1], (self.target - current_agent.position)[0])
        delta_theta = target_theta - agent_theta
        delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))
        normalized_delta_theta = delta_theta / math.pi

        # Obstacle States
        obstacles = current_agent.uavs_in_range(list(self.uavs.values()))
        
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

    def uavs_in_range(self, uav_list):       
        uavs = []
        relative_distances = []
        for i in range(len(uav_list)):
            target_agent = uav_list[i]
            if target_agent == self:
                continue
            distance = np.linalg.norm(target_agent.position - self.position)
            if distance < UAV_SENSE_RANGE:
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
    rospy.init_node('uav_collision_avoidance_node')
    rospy.loginfo('UAV Collision Node Start...')
    rospy.loginfo('Using {}'.format(torch.device("cuda" if torch.cuda.is_available() else "cpu")) )

    # Go to class functions that do all the heavy lifting. Do error checking.    
    try:          
        node = UAVCollisionAvoidanceNode(rospy.get_namespace().split('/')[1])
    except rospy.ROSInterruptException:
        pass

    rospy.spin()


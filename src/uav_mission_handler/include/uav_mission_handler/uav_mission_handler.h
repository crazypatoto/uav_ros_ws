#ifndef UAV_MISSION_HANDLER_H
#define UAV_MISSION_HANDLER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

class UAVMissionHandler
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Subscribers
    ros::Subscriber waypointArrivedSub_;

    // Publishers            
    ros::Publisher waypointPub_;

public:
    UAVMissionHandler(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~UAVMissionHandler();
};

#endif
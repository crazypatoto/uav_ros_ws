#ifndef UAV_MISSION_HANDLER_H
#define UAV_MISSION_HANDLER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_msgs/State.h>
#include <uav_msgs/Takeoff.h>
#include <uav_msgs/Land.h>

using namespace std;

class UAVMissionHandler
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Subscribers
    ros::Subscriber controllerStateSub_;

    // Publishers
    ros::Publisher waypointPub_;

    // Service Clients
    ros::ServiceClient takeoffClient_;
    ros::ServiceClient landClient_;

    // Callbacks
    void controllerStateCallback(const uav_msgs::State::ConstPtr &msg);
    void missionLoopCallback(const ros::TimerEvent &event);

    // Timers
    ros::Timer missionLoopTimer_;

    // Variables
    uav_msgs::State uavState_;

public:
    UAVMissionHandler(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~UAVMissionHandler();
};

#endif
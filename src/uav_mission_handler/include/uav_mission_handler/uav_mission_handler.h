#ifndef UAV_MISSION_HANDLER_H
#define UAV_MISSION_HANDLER_H

#include <iostream>
#include <filesystem>
#include <string>
#include <fstream>
#include <streambuf>
#include <memory>
#include <queue>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <uav_msgs/State.h>
#include <uav_msgs/Takeoff.h>
#include <uav_msgs/Land.h>
#include <uav_msgs/GoHome.h>
#include <uav_msgs/Mission.h>
#include <uav_msgs/Order.h>
#include <uav_msgs/EnqueueOrder.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_ros_link_attacher/Attach.h>

using namespace std;

class UAVMissionHandler
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Subscribers
    ros::Subscriber controllerStateSub_;
    ros::Subscriber uavPoseSub_;

    // Publishers
    ros::Publisher waypointPub_;
    ros::Publisher missionCompletePub_;
    ros::Publisher orderCompletePub_;

    // Service Clients
    ros::ServiceClient takeoffClient_;
    ros::ServiceClient landClient_;
    ros::ServiceClient goHomeClient_;
    ros::ServiceClient spawnModelClient_;
    ros::ServiceClient linkAttachClient_;
    ros::ServiceClient linkDetachClient_;

    // Service Servers
    ros::ServiceServer orderReceiveServer_;

    // Timers
    ros::Timer missionLoopTimer_;

    // Callbacks
    void controllerStateCallback(const uav_msgs::State::ConstPtr &msg);
    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void missionLoopCallback(const ros::TimerEvent &event);
    bool orderReceivedCallback(uav_msgs::EnqueueOrder::Request &req, uav_msgs::EnqueueOrder::Response &res);

    // Functions
    void spawnPackageModel(int packageID, int position);
    void attachPackage(int packageID);
    void detachPackage(int packageID);
    void startNewOrder();
    void takeoff();
    void land();
    void goHome();
    void goTo(geometry_msgs::Vector3 destination);
    void publishFinishedMission(uav_msgs::Mission &mission);
    void publishFinishedOrder(uav_msgs::Order &order);

    // Variables
    bool isDelivering_ = false;
    int uavID_;
    double travelAltitude_;
    uav_msgs::State uavState_;
    geometry_msgs::Pose uavPose_;
    string package_model_path_;
    string packageNamePrefix_;
    std::queue<uav_msgs::Order> orderQueue_;
    uav_msgs::Order currentOrder_;
    uint8_t currentMissionIndex_;

public:
    UAVMissionHandler(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~UAVMissionHandler();
};

#endif
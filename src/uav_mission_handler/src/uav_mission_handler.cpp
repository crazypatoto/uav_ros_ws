#include "uav_mission_handler/uav_mission_handler.h"

using namespace std;

UAVMissionHandler::UAVMissionHandler(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
{
    // Initialize subscribers
    controllerStateSub_ = nh_.subscribe("uav_controller/state", 1, &UAVMissionHandler::controllerStateCallback, this, ros::TransportHints().tcpNoDelay());

    // Initialize publishers
    waypointPub_ = nh_.advertise<geometry_msgs::PoseStamped>("uav_controller/target_waypoint", 1);

    // Initialize service clients
    takeoffClient_ = nh_.serviceClient<uav_msgs::TakeoffCommand>("uav_controller/takeoff");

    // Initialize service servers
    // takeoffServer_ = nh_.advertiseService("uav_controller/takeoff", &UAVController::takeoffServiceCallback, this);

    // Initialize timers
    missionLoopTimer_ = nh_.createTimer(ros::Duration(0.1), &UAVMissionHandler::missionLoopCallback, this); // Define timer for constant loop rate
}

void UAVMissionHandler::controllerStateCallback(const uav_msgs::State::ConstPtr &msg)
{
    uavState_ = *msg;
}

void UAVMissionHandler::missionLoopCallback(const ros::TimerEvent &event)
{
    
}
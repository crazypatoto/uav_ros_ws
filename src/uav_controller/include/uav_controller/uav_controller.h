#ifndef GEOMETRIC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

enum class MAV_STATE
{
    MAV_STATE_UNINIT = 0,
    MAV_STATE_BOOT = 1,
    MAV_STATE_CALIBRATING = 2,
    MAV_STATE_STANDBY = 3,
    MAV_STATE_ACTIVE = 4,
    MAV_STATE_CRITICAL = 5,
    MAV_STATE_EMERGENCY = 6,
    MAV_STATE_POWEROFF = 7,
    MAV_STATE_FLIGHT_TERMINATION = 8,
};

class UAVController
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Subscribers
    ros::Subscriber mavStateSub_;
    ros::Subscriber groundTruthSub_;

    // Publishers
    ros::Publisher companionStatusPub_;
    ros::Publisher bodyRatePub_;

    // Service Clients
    ros::ServiceClient armingClient_;
    ros::ServiceClient setModeClient_;

    // Service Servers

    // Timers
    ros::Timer statusLoopTimer_;
    ros::Timer controlLoopTimer_;

    // Local Variables
    MAV_STATE companionState_ = MAV_STATE::MAV_STATE_UNINIT;
    mavros_msgs::State currentMavState_;
    mavros_msgs::SetMode mavSetMode_;
    mavros_msgs::CommandBool mavArmCommand_;

    ros::Time lastRequest_;
    geometry_msgs::Pose homePose_;
    bool homePoseReceived = false;

    // Callbacks
    void mavstateCallback(const mavros_msgs::State::ConstPtr &msg);
    void groundTruthCallback(const nav_msgs::Odometry &msg);
    void statusLoopCallback(const ros::TimerEvent &event);
    void controlLoopCallback(const ros::TimerEvent &event);

    // Functions
    void publishCompanionState();

    enum FlightState
    {
        WAITING_FCU_CONNECTION = 0,
        WAITING_HOME_POSE = 1,        
        FLYING = 2,
        LANDING = 3,
        LANDED = 4
    } nodeState;

    template <class T>
    void waitForPredicate(const T *pred, const std::string &msg, double hz = 2.0)
    {
        ros::Rate pause(hz);
        ROS_INFO_STREAM(msg);
        while (ros::ok() && !(*pred))
        {
            ros::spinOnce();
            pause.sleep();
        }
    };

public:
    UAVController(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
};

#endif
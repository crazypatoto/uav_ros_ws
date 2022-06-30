#ifndef GEOMETRIC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_H

#define TARGET_REACH_THRESHOLD (0.05)

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <uav_msgs/State.h>
#include <ruckig_ros/ruckig.hpp>

#include <Eigen/Dense>

#include "uav_controller/common.h"
#include "uav_controller/cubicpolytraj.h"
#include "uav_controller/quinticpolytraj.h"
#include "uav_controller/TakeoffCommand.h"

using namespace std;
using namespace Eigen;
using namespace ruckig;

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
    ros::Subscriber waypointSub_;

    // Publishers
    ros::Publisher controllerStatePub_;
    ros::Publisher companionStatusPub_;
    ros::Publisher bodyRatePub_;
    ros::Publisher posePub_;
    ros::Publisher referencePosePub_;
    ros::Publisher posehistoryPub_;
    ros::Publisher referenceTrajPub_;

    // Service Clients
    ros::ServiceClient armingClient_;
    ros::ServiceClient setModeClient_;

    // Service Servers
    ros::ServiceServer takeoffServer_;

    // Timers
    ros::Timer statusLoopTimer_;
    ros::Timer controlLoopTimer_;

    // Local Variables
    MAV_STATE companionState_ = MAV_STATE::MAV_STATE_UNINIT;
    mavros_msgs::State currentMavState_;
    mavros_msgs::SetMode mavSetMode_;
    mavros_msgs::CommandBool mavArmCommand_;
    geometry_msgs::Pose homePose_;
    ros::Time lastMavCommandRequest_;
    Eigen::Vector3d desired_acc;
    Eigen::Vector3d targetPos_, targetVel_, targetAcc_;
    Eigen::Vector3d targetPos_prev_, targetVel_prev_;
    Eigen::Vector3d targetWayPointPos_;
    Eigen::Vector3d mavPos_, mavVel_, mavVelPrev_, mavRate_, mavAcc_;
    Eigen::Vector4d mavAtt_;
    Eigen::Vector3d g_;
    Eigen::Vector3d drag_;
    Eigen::Vector3d Kpos_, Kvel_;
    Eigen::Vector4d q_des;
    Eigen::Vector4d cmdBodyRate_; //{wx, wy, wz, Thrust}
    bool homePoseReceived_ = false;
    bool readyToTakeoff_ = false;
    bool waypointArrived_ = false;
    bool autoTakeoff_;
    bool velocity_yaw_;
    double initTargetPos_z_;
    double targetYaw_;
    double dx_, dy_, dz_;
    double max_fb_acc_;
    double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_;
    double attctrl_tau_;
    double norm_thrust_const_, norm_thrust_offset_;
    double trajectory_max_vel_x_, trajectory_max_vel_y_, trajectory_max_vel_z_;
    double trajectory_max_acc_x_, trajectory_max_acc_y_, trajectory_max_acc_z_;
    double trajectory_max_jerk_x_, trajectory_max_jerk_y_, trajectory_max_jerk_z_;
    std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
    int posehistory_window_;

    Ruckig<3> ruckig_{0.010};         // Number DoFs; control cycle in [s]
    InputParameter<3> ruckigInput_;   // Number DoFs
    OutputParameter<3> ruckigOutput_; // Number DoFs
    ruckig::Result ruckigResult_;
    double goundTruth_dt_;
    ros::Time groundTruth_last_time_;

    // Callbacks
    void mavstateCallback(const mavros_msgs::State::ConstPtr &msg);
    void groundTruthCallback(const nav_msgs::Odometry &msg);
    void waypointCallback(const geometry_msgs::PoseStamped &msg);
    void statusLoopCallback(const ros::TimerEvent &event);
    void controlLoopCallback(const ros::TimerEvent &event);
    bool takeoffServiceCallback(uav_controller::TakeoffCommand::Request &req, uav_controller::TakeoffCommand::Response &res);

    // Functions
    void publishCompanionState();
    Eigen::Vector3d controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc);
    Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);
    void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des);
    Eigen::Vector3d poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error);
    Eigen::Vector4d geometric_attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att);
    void pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude);
    void pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude);
    void appendPoseHistory();
    void pubPoseHistory();
    void pubRefTraj();
    void pubWaypointArrived(const bool arrived);
    void pubControllerState();

    enum ContorllerState
    {
        WAITING_FCU_CONNECTION = 0,
        WAITING_HOME_POSE = 1,
        WAITING_TAKEOFF_COMMAND = 2,
        TAKING_OFF = 3,
        FLYING = 4,
        LANDING = 5,
    } controllerState_;

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
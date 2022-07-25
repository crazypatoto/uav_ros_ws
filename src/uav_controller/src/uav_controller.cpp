#include "uav_controller/uav_controller.h"

using namespace Eigen;
using namespace std;

UAVController::UAVController(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
{
    // Get current namespace
    namespace_ = ros::this_node::getNamespace();
    namespace_.erase(remove(namespace_.begin(), namespace_.end(), '/'), namespace_.end()); // remove '/' from string
    ROS_INFO_STREAM("Current Namespace: " << namespace_);

    // Initialize subscribers
    mavStateSub_ = nh_.subscribe("mavros/state", 1, &UAVController::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
    groundTruthSub_ = nh_.subscribe("ground_truth/state", 1, &UAVController::groundTruthCallback, this, ros::TransportHints().tcpNoDelay());
    waypointSub_ = nh_.subscribe("uav_controller/target_waypoint", 1, &UAVController::waypointCallback, this, ros::TransportHints().tcpNoDelay());
    cmdVelSub_ = nh_.subscribe("uav_controller/cmd_vel", 1, &UAVController::cmdVelCallback, this, ros::TransportHints().tcpNoDelay());

    // Initialize publishers
    controllerStatePub_ = nh_.advertise<uav_msgs::State>("uav_controller/state", 1);
    companionStatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);
    bodyRatePub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1);
    posePub_ = nh_.advertise<geometry_msgs::PoseStamped>("uav_controller/pose", 1);
    referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("uav_controller/reference/pose", 1);
    posehistoryPub_ = nh_.advertise<nav_msgs::Path>("uav_controller/path", 10);
    referenceTrajPub_ = nh_.advertise<nav_msgs::Path>("uav_controller/reference/trajectory", 10);

    // Initialize service clients
    armingClient_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    setModeClient_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ucaClient_ = nh_.serviceClient<uav_msgs::UCASelectAction>("uav_collision_avoidance/select_action");
    uavsInRangeClient_ = nh_.serviceClient<uav_msgs::UAVsInRange>("uav_collision_avoidance/uavs_in_range");

    // Initialize service servers
    takeoffServer_ = nh_.advertiseService("uav_controller/takeoff", &UAVController::takeoffServiceCallback, this);
    landServer_ = nh_.advertiseService("uav_controller/land", &UAVController::landServiceCallback, this);
    goHomeServer_ = nh_.advertiseService("uav_controller/go_home", &UAVController::goHomeServiceCallback, this);

    // Initialize Parameters
    nh_private_.param<bool>("auto_takeoff", autoTakeoff_, true);
    nh_private_.param<bool>("velocity_yaw", velocity_yaw_preset_, false);
    nh_private_.param<double>("target_yaw", targetYaw_, 0.0);
    nh_private_.param<double>("initTargetPos_z", initTargetPos_z_, 2);
    nh_private_.param<double>("drag_dx", dx_, 0.0);
    nh_private_.param<double>("drag_dy", dy_, 0.0);
    nh_private_.param<double>("drag_dz", dz_, 0.0);
    nh_private_.param<double>("max_acc", max_fb_acc_, 8.0);
    nh_private_.param<double>("attctrl_constant", attctrl_tau_, 0.1);
    nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.063); // 1 / max acceleration
    nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.1);    // 1 / max acceleration
    nh_private_.param<int>("posehistory_window", posehistory_window_, 1000);
    nh_private_.param<bool>("collision_avoidance_enabled", collision_avoidance_enabled_, false);
    nh_private_.param<double>("collision_avoidance_maximum_acceleration", uca_max_acc_, 5);
    // Trajectory Generation Parameters
    nh_private_.param<double>("trajectory_max_vel_x", trajectory_max_vel_x_, 7.0);
    nh_private_.param<double>("trajectory_max_vel_y", trajectory_max_vel_y_, 7.0);
    nh_private_.param<double>("trajectory_max_vel_z", trajectory_max_vel_z_, 9.0);
    nh_private_.param<double>("trajectory_max_acc_x", trajectory_max_acc_x_, 4.0);
    nh_private_.param<double>("trajectory_max_acc_y", trajectory_max_acc_y_, 4.0);
    nh_private_.param<double>("trajectory_max_acc_z", trajectory_max_acc_z_, 3.0);
    nh_private_.param<double>("trajectory_max_jerk_x", trajectory_max_jerk_x_, 4.0);
    nh_private_.param<double>("trajectory_max_jerk_y", trajectory_max_jerk_y_, 4.0);
    nh_private_.param<double>("trajectory_max_jerk_z", trajectory_max_jerk_z_, 3.0);
    // PD Position Controller Parameters
    nh_private_.param<double>("Kp_x", Kpos_x_, 3.0);
    nh_private_.param<double>("Kp_y", Kpos_y_, 3.0);
    nh_private_.param<double>("Kp_z", Kpos_z_, 3.0);
    nh_private_.param<double>("Kv_x", Kvel_x_, 5.0);
    nh_private_.param<double>("Kv_y", Kvel_y_, 5.0);
    nh_private_.param<double>("Kv_z", Kvel_z_, 3.0);
    // PID Velocity Controller Parameters
    nh_private_.param<double>("P_x", P_x_, 3.0);
    nh_private_.param<double>("P_y", P_y_, 3.0);
    nh_private_.param<double>("P_z", P_z_, 3.0);
    nh_private_.param<double>("I_x", I_x_, 0.3);
    nh_private_.param<double>("I_y", I_y_, 0.3);
    nh_private_.param<double>("I_z", I_z_, 0.3);
    nh_private_.param<double>("D_x", D_x_, 0.01);
    nh_private_.param<double>("D_y", D_y_, 0.01);
    nh_private_.param<double>("D_z", D_z_, 0.01);

    targetPos_ << 0, 0, initTargetPos_z_; // Initial Position
    targetVel_ << 0.0, 0.0, 0.0;
    mavPos_ << 0.0, 0.0, 0.0;
    mavVel_ << 0.0, 0.0, 0.0;
    mavAtt_ << 1.0, 0.0, 0.0, 0.0;
    g_ << 0.0, 0.0, -9.807; // Earth acceleration
    drag_ << dx_, dy_, dz_;
    Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
    Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
    P_ << P_x_, P_y_, P_z_;
    I_ << I_x_, I_y_, I_z_;
    D_ << D_x_, D_y_, D_z_;
    ucaMaxAcc_ << uca_max_acc_, uca_max_acc_, uca_max_acc_;

    // Initialize timers
    statusLoopTimer_ = nh_.createTimer(ros::Duration(0.1), &UAVController::statusLoopCallback, this);    // Define timer for constant loop rate
    controlLoopTimer_ = nh_.createTimer(ros::Duration(0.01), &UAVController::controlLoopCallback, this); // Define timer for constant loop rate
}

void UAVController::mavstateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    currentMavState_ = *msg;
}

void UAVController::groundTruthCallback(const nav_msgs::Odometry &msg)
{
    if (!homePoseReceived_ && controllerState_ == WAITING_HOME_POSE)
    {
        homePoseReceived_ = true;
        homePose_ = msg.pose.pose;
        ROS_INFO_STREAM("Home pose initialized to: " << homePose_);
    }
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id = "world";
    poseMsg.header.stamp = msg.header.stamp;
    poseMsg.pose = msg.pose.pose;
    posePub_.publish(poseMsg);
    mavPos_ = toEigen(msg.pose.pose.position);
    mavAtt_(0) = msg.pose.pose.orientation.w;
    mavAtt_(1) = msg.pose.pose.orientation.x;
    mavAtt_(2) = msg.pose.pose.orientation.y;
    mavAtt_(3) = msg.pose.pose.orientation.z;

    mavVel_ = toEigen(msg.twist.twist.linear);
    mavRate_ = toEigen(msg.twist.twist.angular);

    ros::Time now = msg.header.stamp;
    goundTruth_dt_ = (now - groundTruth_last_time_).toSec();

    mavAcc_ = (mavVel_ - mavVelPrev_) / goundTruth_dt_;

    groundTruth_last_time_ = now;
    mavVelPrev_ = mavVel_;

    // ROS_INFO("dt = %0.3f; Acc: x = %+0.2f, %+0.2f, %+0.2f", goundTruth_dt_, mavAcc_(0), mavAcc_(0), mavAcc_(0));
}

void UAVController::waypointCallback(const geometry_msgs::PoseStamped &msg)
{
    targetWayPointPos_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

    // ROS_INFO_STREAM("targetWayPointPos_: " << targetWayPointPos_);
    // ROS_INFO_STREAM("mavPos_: " << mavPos_);
    // ROS_INFO_STREAM("mavVel_: " << mavVel_);
    // ROS_INFO_STREAM("mavAcc_: " << mavAcc_);
    travelToTargetWaypoint();
    pubRefTraj();
}

void UAVController::cmdVelCallback(const geometry_msgs::TwistStamped &msg)
{
    cmdVelReceived_ = true;
    targetVel_ << msg.twist.linear.x, msg.twist.linear.y, 0;
}

void UAVController::statusLoopCallback(const ros::TimerEvent &event)
{
    if (readyToTakeoff_ && (controllerState_ == WAITING_TAKEOFF_COMMAND || controllerState_ == TAKING_OFF))
    {
        controllerState_ = TAKING_OFF;
        mavArmCommand_.request.value = true;
        mavSetMode_.request.custom_mode = "OFFBOARD"; // See all available modes at http://wiki.ros.org/mavros/CustomModes
        if (currentMavState_.mode != "OFFBOARD" && (ros::Time::now() - lastMavCommandRequest_ > ros::Duration(5.0)))
        {
            if (setModeClient_.call(mavSetMode_) && mavSetMode_.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            lastMavCommandRequest_ = ros::Time::now();
        }
        else
        {
            if (!currentMavState_.armed && (ros::Time::now() - lastMavCommandRequest_ > ros::Duration(5.0)))
            {
                if (armingClient_.call(mavArmCommand_) && mavArmCommand_.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                lastMavCommandRequest_ = ros::Time::now();
            }
        }
    }
    publishCompanionState();
    pubControllerState();
}

void UAVController::controlLoopCallback(const ros::TimerEvent &event)
{
    switch (controllerState_)
    {
    case WAITING_FCU_CONNECTION:
        // Wait for FCU connection
        waitForPredicate(&currentMavState_.connected, "Connecting to FCU...", 2);
        ROS_INFO("Connected to FCU");
        controllerState_ = WAITING_HOME_POSE;
        companionState_ = MAV_STATE::MAV_STATE_STANDBY;
        break;
    case WAITING_HOME_POSE:
        waitForPredicate(&homePoseReceived_, "Waiting for home position...", 2);
        ROS_INFO("Got pose! Drone ready to be armed and take off.");
        targetPos_ << homePose_.position.x, homePose_.position.y, initTargetPos_z_;
        controllerState_ = WAITING_TAKEOFF_COMMAND;
        companionState_ = MAV_STATE::MAV_STATE_ACTIVE;
        ROS_INFO("Waiting for takeoff command...");
        break;
    case WAITING_TAKEOFF_COMMAND: // Handled in Status Loop Callback.
        if (autoTakeoff_)
        {
            ROS_INFO("Auto takeoff initiated, taking off!");
            readyToTakeoff_ = true;
            controllerState_ = TAKING_OFF;
        }
    case TAKING_OFF:
        targetPos_ << homePose_.position.x, homePose_.position.y, initTargetPos_z_;
        targetWayPointPos_ << targetPos_;
        desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_, 0);
        computeBodyRateCmd(cmdBodyRate_, desired_acc);
        pubReferencePose(targetPos_, q_des);
        pubRateCommands(cmdBodyRate_, q_des);
        appendPoseHistory();
        pubPoseHistory();

        if ((distance(targetPos_, mavPos_) < TARGET_REACH_POS_THRESHOLD) && (mavVel_.norm() < TARGET_REACH_VEL_THRESHOLD))
        {
            waypointArrived_ = true;
            controllerState_ = FLYING;
        }
        break;
    case FLYING:
        if (!cmdVelReceived_)
        {
            if (collision_avoidance_enabled_)
            {
                isUAVInRange_ = checkIfUAVInRange();
                bool closeToTarget;
                closeToTarget = (targetWayPointPos_ - mavPos_).norm() < MINIMUM_UCA_TRAGET_DISTANCE;
                if ((!isUAVInRange_ || closeToTarget) && avoidingCollisions_) // If there is no longer any UAV in range or close to target, recalculate trajectory and stop avoding UAVs
                {
                    ROS_INFO("Stop avoiding...");
                    avoidingCollisions_ = false;
                    travelToTargetWaypoint();
                    pubRefTraj();
                }
                else if (isUAVInRange_ && !waypointArrived_ && !closeToTarget && !avoidingCollisions_)
                {
                    ROS_INFO("Start avoiding...");
                    avoidingCollisions_ = true;
                    targetVel_prev_ = mavVel_; 
                    targetPos_ << targetWayPointPos_;
                }

                isUAVInRangePrev_ = isUAVInRange_;
            }

            if (!avoidingCollisions_)
            {
                ruckigResult_ = ruckig_.update(ruckigInput_, ruckigOutput_);
                if (ruckigResult_ == Result::Working)
                {
                    targetPos_ << ruckigOutput_.new_position[0], ruckigOutput_.new_position[1], ruckigOutput_.new_position[2];
                    targetVel_ << ruckigOutput_.new_velocity[0], ruckigOutput_.new_velocity[1], ruckigOutput_.new_velocity[2];
                    targetAcc_ << ruckigOutput_.new_acceleration[0], ruckigOutput_.new_acceleration[1], ruckigOutput_.new_acceleration[2];

                    if (velocity_yaw_preset_)
                    {
                        velocity_yaw_ = true;
                    }
                    // ROS_INFO_STREAM("targetPos_: " << targetPos_);
                    // ROS_INFO_STREAM("targetVel_: " << targetVel_);
                    // ROS_INFO_STREAM("targetAcc_: " << targetAcc_);
                    ruckigOutput_.pass_to_input(ruckigInput_);
                }
                else if (ruckigResult_ == Result::Finished)
                {
                    if ((distance(targetWayPointPos_, mavPos_) < TARGET_REACH_POS_THRESHOLD) && (mavVel_.norm() < TARGET_REACH_VEL_THRESHOLD))
                    {
                        waypointArrived_ = true;
                    }
                    velocity_yaw_ = false;
                }
                desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_, 0);
            }
            else // Otherwise get action from AI model
            {
                Eigen::Vector3d deltaVel;
                targetVel_ = chooseAction();  
                deltaVel = (targetVel_ - targetVel_prev_) / 0.01;
                deltaVel = deltaVel.cwiseMax(-ucaMaxAcc_).cwiseMin(ucaMaxAcc_);
                targetVel_ = targetVel_prev_ + (deltaVel * 0.01);
                targetVel_(2) = 0;
                targetVel_prev_ = targetVel_;
                desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_, 1);
            }
        }
        else
        {
            desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_, 1);
        }

        computeBodyRateCmd(cmdBodyRate_, desired_acc);
        pubReferencePose(targetPos_, q_des);
        pubRateCommands(cmdBodyRate_, q_des);
        appendPoseHistory();
        pubPoseHistory();
        break;

    case LANDING:
        // UAV will disarmed in a few seconds after auto landing
        if (currentMavState_.armed == 0)
        {
            ROS_INFO("Landed successfully!");
            autoTakeoff_ = false;
            readyToTakeoff_ = false;
            controllerState_ = WAITING_TAKEOFF_COMMAND;
        }
        break;
    }
}

bool UAVController::takeoffServiceCallback(uav_msgs::Takeoff::Request &req, uav_msgs::Takeoff::Response &res)
{
    if (controllerState_ != WAITING_TAKEOFF_COMMAND)
    {
        res.success = false;
        return true;
    }

    initTargetPos_z_ = req.absoluteAltitude;
    readyToTakeoff_ = true;
    res.success = true;

    return true;
}

bool UAVController::landServiceCallback(uav_msgs::Land::Request &req, uav_msgs::Land::Response &res)
{
    res.success = false;
    if (!waypointArrived_)
    {
        return true;
    }

    mavSetMode_.request.custom_mode = "AUTO.LAND";
    if (setModeClient_.call(mavSetMode_) && mavSetMode_.response.mode_sent)
    {
        ROS_INFO("AUTO.LAND Mode Set.");
        ROS_INFO("Landing...");
        controllerState_ = LANDING;
        res.success = true;
    }
    lastMavCommandRequest_ = ros::Time::now();

    return true;
}

bool UAVController::goHomeServiceCallback(uav_msgs::GoHome::Request &req, uav_msgs::GoHome::Response &res)
{
    res.success = false;
    if (controllerState_ != FLYING)
    {
        return true;
    }

    targetWayPointPos_ << homePose_.position.x, homePose_.position.y, initTargetPos_z_;

    travelToTargetWaypoint();
    pubRefTraj();

    res.success = true;
    return true;
}

void UAVController::publishCompanionState()
{
    mavros_msgs::CompanionProcessStatus msg;

    msg.header.stamp = ros::Time::now();
    msg.component = 196; // MAV_COMPONENT_ID_AVOIDANCE
    msg.state = (int)companionState_;

    companionStatusPub_.publish(msg);
}

Eigen::Vector3d UAVController::controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc, const int mode)
{
    /// Compute BodyRate commands using differential flatness
    /// Controller based on Faessler 2017
    /// Mode 0: Original position controller; Mode 1: Simple Cascade PID position-velocity controller (position control in Z axis only)

    const Eigen::Vector3d a_ref = target_acc;
    if (velocity_yaw_)
    {
        if (target_vel(0) != 0.0 || target_vel(1) != 0.0)
        {
            targetYaw_ = atan2(mavVel_(1), mavVel_(0));
        }
    }

    Eigen::Vector3d pos_error = mavPos_ - target_pos;
    const Eigen::Vector3d vel_error = mavVel_ - target_vel;

    // Position Controller
    Eigen::Vector3d a_fb;
    if (mode)
    {
        a_fb = posvelcontroller();
    }
    else
    {
        a_fb = poscontroller(pos_error, vel_error);
    }

    // Rotor Drag compensation
    const Eigen::Vector4d q_ref = acc2quaternion(a_ref - g_, targetYaw_);
    const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);
    const Eigen::Vector3d a_rd = R_ref * drag_.asDiagonal() * R_ref.transpose() * target_vel; // Rotor drag

    // Reference acceleration
    const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - g_;

    return a_des;
}

Eigen::Vector4d UAVController::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw)
{
    Eigen::Vector4d quat;
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
    Eigen::Matrix3d rotmat;

    proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

    zb_des = vector_acc / vector_acc.norm();
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
    xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

    rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
    quat = rot2Quaternion(rotmat);
    return quat;
}

void UAVController::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des)
{
    // Reference attitude
    q_des = acc2quaternion(a_des, targetYaw_);

    bodyrate_cmd = geometric_attcontroller(q_des, a_des, mavAtt_); // Calculate BodyRate
}

Eigen::Vector3d UAVController::poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error)
{
    Eigen::Vector3d a_fb = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error; // feedforward term for trajectory error

    if (a_fb.norm() > max_fb_acc_)
        a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb; // Clip acceleration if reference is too large

    return a_fb;
}

Eigen::Vector3d UAVController::posvelcontroller()
{
    static Eigen::Vector3d vel_int, vel_error_last;
    Eigen::Vector3d pos_error = targetPos_ - mavPos_;

    pos_error << 0, 0, pos_error(2); // Ignore X, Y axis positional error

    Eigen::Vector3d vel_sp_position = Kpos_.asDiagonal() * -pos_error;
    Eigen::Vector3d vel_sp = targetVel_ + vel_sp_position;
    Eigen::Vector3d vel_error = vel_sp - mavVel_;
    Eigen::Vector3d a_fb = P_.asDiagonal() * vel_error + vel_int + D_.asDiagonal() * (vel_error - vel_error_last) / 0.01;

    vel_int += I_.asDiagonal() * vel_error * 0.01;
    vel_error_last = vel_error;

    if (a_fb.norm() > max_fb_acc_)
        a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb; // Clip acceleration if reference is too large

    return a_fb;
}

Eigen::Vector4d UAVController::geometric_attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att)
{
    // Geometric attitude controller
    // Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control
    // of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.
    // The original paper inputs moment commands, but for offboard control, angular rate commands are sent

    Eigen::Vector4d ratecmd;
    Eigen::Matrix3d rotmat;   // Rotation matrix of current attitude
    Eigen::Matrix3d rotmat_d; // Rotation matrix of desired attitude
    Eigen::Vector3d error_att;

    rotmat = quat2RotMatrix(curr_att);
    rotmat_d = quat2RotMatrix(ref_att);

    error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat_d);
    ratecmd.head(3) = (2.0 / attctrl_tau_) * error_att;
    rotmat = quat2RotMatrix(mavAtt_);
    const Eigen::Vector3d zb = rotmat.col(2);
    ratecmd(3) =
        std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_)); // Calculate thrust

    return ratecmd;
}

void UAVController::pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude)
{
    geometry_msgs::PoseStamped msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.pose.position.x = target_position(0);
    msg.pose.position.y = target_position(1);
    msg.pose.position.z = target_position(2);
    msg.pose.orientation.w = target_attitude(0);
    msg.pose.orientation.x = target_attitude(1);
    msg.pose.orientation.y = target_attitude(2);
    msg.pose.orientation.z = target_attitude(3);
    referencePosePub_.publish(msg);
}

void UAVController::pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude)
{
    mavros_msgs::AttitudeTarget msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.body_rate.x = cmd(0);
    msg.body_rate.y = cmd(1);
    msg.body_rate.z = cmd(2);
    msg.type_mask = 128; // Ignore orientation messages
    msg.orientation.w = target_attitude(0);
    msg.orientation.x = target_attitude(1);
    msg.orientation.y = target_attitude(2);
    msg.orientation.z = target_attitude(3);
    msg.thrust = cmd(3);

    bodyRatePub_.publish(msg);
}

void UAVController::appendPoseHistory()
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "world";
    pose_msg.pose.orientation.w = mavAtt_(0);
    pose_msg.pose.orientation.x = mavAtt_(1);
    pose_msg.pose.orientation.y = mavAtt_(2);
    pose_msg.pose.orientation.z = mavAtt_(3);
    pose_msg.pose.position.x = mavPos_(0);
    pose_msg.pose.position.y = mavPos_(1);
    pose_msg.pose.position.z = mavPos_(2);
    posehistory_vector_.insert(posehistory_vector_.begin(), pose_msg);
    if (posehistory_vector_.size() > posehistory_window_)
    {
        posehistory_vector_.pop_back();
    }
}

void UAVController::pubPoseHistory()
{
    nav_msgs::Path msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.poses = posehistory_vector_;

    posehistoryPub_.publish(msg);
}

void UAVController::pubRefTraj()
{
    // Generate Static Reference Trajectory and Publish
    Ruckig<3> otg;
    Trajectory<3> trajectory;
    otg.calculate(ruckigInput_, trajectory);

    std::vector<geometry_msgs::PoseStamped> traj;

    for (double t = 0; t < trajectory.get_duration(); t += 0.1)
    {
        geometry_msgs::PoseStamped pose_msg;
        std::array<double, 3> newPos, newVel, newAcc;
        trajectory.at_time(t, newPos, newVel, newAcc);
        pose_msg.header.frame_id = "world";

        pose_msg.pose.position.x = newPos[0];
        pose_msg.pose.position.y = newPos[1];
        pose_msg.pose.position.z = newPos[2];
        traj.insert(traj.begin(), pose_msg);
    }

    nav_msgs::Path pathMsg;

    pathMsg.header.stamp = ros::Time::now();
    pathMsg.header.frame_id = "world";
    pathMsg.poses = traj;

    referenceTrajPub_.publish(pathMsg);
}

void UAVController::pubControllerState()
{
    uav_msgs::State msg;
    msg.header.stamp = ros::Time::now();
    msg.state = controllerState_;
    msg.currentWaypoint.position.x = targetWayPointPos_(0);
    msg.currentWaypoint.position.y = targetWayPointPos_(1);
    msg.currentWaypoint.position.z = targetWayPointPos_(2);
    msg.waypointArrived = waypointArrived_;

    controllerStatePub_.publish(msg);
}

void UAVController::travelToTargetWaypoint()
{
    waypointArrived_ = false;

    ruckigInput_.current_position = {mavPos_(0), mavPos_(1), mavPos_(2)};
    ruckigInput_.current_velocity = {mavVel_(0), mavVel_(1), mavVel_(2)};
    ruckigInput_.current_acceleration = {0, 0, 0}; // use uav acceleration for best result
    ruckigInput_.target_position = {targetWayPointPos_(0), targetWayPointPos_(1), targetWayPointPos_(2)};
    ruckigInput_.target_velocity = {0.0, 0.0, 0.0};
    ruckigInput_.target_acceleration = {0.0, 0.0, 0.0};
    ruckigInput_.max_velocity = {trajectory_max_vel_x_, trajectory_max_vel_y_, trajectory_max_vel_z_};
    ruckigInput_.max_acceleration = {trajectory_max_acc_x_, trajectory_max_acc_y_, trajectory_max_acc_z_};
    ruckigInput_.max_jerk = {trajectory_max_jerk_x_, trajectory_max_jerk_y_, trajectory_max_jerk_z_};
}

Eigen::Vector3d UAVController::chooseAction()
{
    Eigen::Vector3d vel;
    uav_msgs::UCASelectAction srv;
    srv.request.uavNamespace = namespace_;
    srv.request.target.x = targetPos_(0);
    srv.request.target.y = targetPos_(1);
    srv.request.target.z = targetPos_(2);
    if (ucaClient_.call(srv) && srv.response.success)
    {
        vel << srv.response.action.x, srv.response.action.y, srv.response.action.z;
    }

    return vel;
}

bool UAVController::checkIfUAVInRange()
{
    uav_msgs::UAVsInRange srv;
    srv.request.uavNamespace = namespace_;
    if (uavsInRangeClient_.call(srv) && srv.response.success)
    {
        return srv.response.uavs.size() > 0;
    }
    return false;
}

void UAVController::dynamicReconfigureCallback(uav_controller::UAVControllerConfig &config, uint32_t level)
{
    if (max_fb_acc_ != config.max_acc)
    {
        max_fb_acc_ = config.max_acc;
        ROS_INFO("Reconfigure request : max_acc = %.2f ", config.max_acc);
    }
    else if (Kpos_x_ != config.Kp_x)
    {
        Kpos_x_ = config.Kp_x;
        ROS_INFO("Reconfigure request : Kp_x  = %.2f  ", config.Kp_x);
    }
    else if (Kpos_y_ != config.Kp_y)
    {
        Kpos_y_ = config.Kp_y;
        ROS_INFO("Reconfigure request : Kp_y  = %.2f  ", config.Kp_y);
    }
    else if (Kpos_z_ != config.Kp_z)
    {
        Kpos_z_ = config.Kp_z;
        ROS_INFO("Reconfigure request : Kp_z  = %.2f  ", config.Kp_z);
    }
    else if (Kvel_x_ != config.Kv_x)
    {
        Kvel_x_ = config.Kv_x;
        ROS_INFO("Reconfigure request : Kv_x  = %.2f  ", config.Kv_x);
    }
    else if (Kvel_y_ != config.Kv_y)
    {
        Kvel_y_ = config.Kv_y;
        ROS_INFO("Reconfigure request : Kv_y =%.2f  ", config.Kv_y);
    }
    else if (Kvel_z_ != config.Kv_z)
    {
        Kvel_z_ = config.Kv_z;
        ROS_INFO("Reconfigure request : Kv_z  = %.2f  ", config.Kv_z);
    }
    else if (P_x_ != config.velP_x)
    {
        P_x_ = config.velP_x;
        ROS_INFO("Reconfigure request : P_x  = %.2f  ", config.velP_x);
    }
    else if (P_y_ != config.velP_y)
    {
        P_y_ = config.velP_y;
        ROS_INFO("Reconfigure request : P_y  = %.2f  ", config.velP_y);
    }
    else if (P_z_ != config.velP_z)
    {
        P_z_ = config.velP_z;
        ROS_INFO("Reconfigure request : P_z  = %.2f  ", config.velP_z);
    }
    else if (I_x_ != config.velI_x)
    {
        I_x_ = config.velI_x;
        ROS_INFO("Reconfigure request : I_x  = %.2f  ", config.velI_x);
    }
    else if (I_y_ != config.velI_y)
    {
        I_y_ = config.velI_y;
        ROS_INFO("Reconfigure request : I_y  = %.2f  ", config.velI_y);
    }
    else if (I_z_ != config.velI_z)
    {
        I_z_ = config.velI_z;
        ROS_INFO("Reconfigure request : I_z  = %.2f  ", config.velI_z);
    }
    else if (D_x_ != config.velD_x)
    {
        D_x_ = config.velD_x;
        ROS_INFO("Reconfigure request : D_x  = %.2f  ", config.velD_x);
    }
    else if (D_y_ != config.velD_y)
    {
        D_y_ = config.velD_y;
        ROS_INFO("Reconfigure request : D_y  = %.2f  ", config.velD_y);
    }
    else if (D_z_ != config.velD_z)
    {
        D_z_ = config.velD_z;
        ROS_INFO("Reconfigure request : D_z  = %.2f  ", config.velD_z);
    }

    Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
    Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
    P_ << P_x_, P_y_, P_z_;
    I_ << I_x_, I_y_, I_z_;
    D_ << D_x_, D_y_, D_z_;
}

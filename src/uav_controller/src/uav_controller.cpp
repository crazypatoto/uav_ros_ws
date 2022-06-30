#include "uav_controller/uav_controller.h"

using namespace Eigen;
using namespace std;

UAVController::UAVController(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
{
    // Initialize subscribers
    mavStateSub_ = nh_.subscribe("mavros/state", 1, &UAVController::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
    groundTruthSub_ = nh_.subscribe("ground_truth/state", 1, &UAVController::groundTruthCallback, this, ros::TransportHints().tcpNoDelay());
    waypointSub_ = nh_.subscribe("uav_controller/target_waypoint", 1, &UAVController::waypointCallback, this, ros::TransportHints().tcpNoDelay());

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

    // Initialize service servers
    takeoffServer_ = nh_.advertiseService("uav_controller/takeoff", &UAVController::takeoffServiceCallback, this);

    // Initialize timers
    statusLoopTimer_ = nh_.createTimer(ros::Duration(0.1), &UAVController::statusLoopCallback, this);    // Define timer for constant loop rate
    controlLoopTimer_ = nh_.createTimer(ros::Duration(0.01), &UAVController::controlLoopCallback, this); // Define timer for constant loop rate

    // Initialize Parameters
    nh_private_.param<bool>("auto_takeoff", autoTakeoff_, true);
    nh_private_.param<bool>("velocity_yaw", velocity_yaw_, false);
    nh_private_.param<double>("target_yaw", targetYaw_, 0.0);
    nh_private_.param<double>("initTargetPos_z", initTargetPos_z_, 2);
    nh_private_.param<double>("drag_dx", dx_, 0.0);
    nh_private_.param<double>("drag_dy", dy_, 0.0);
    nh_private_.param<double>("drag_dz", dz_, 0.0);
    nh_private_.param<double>("max_acc", max_fb_acc_, 8.0);
    nh_private_.param<double>("Kp_x", Kpos_x_, 5.0);
    nh_private_.param<double>("Kp_y", Kpos_y_, 5.0);
    nh_private_.param<double>("Kp_z", Kpos_z_, 5.0);
    nh_private_.param<double>("Kv_x", Kvel_x_, 1.5);
    nh_private_.param<double>("Kv_y", Kvel_y_, 1.5);
    nh_private_.param<double>("Kv_z", Kvel_z_, 3.3);
    nh_private_.param<double>("attctrl_constant", attctrl_tau_, 0.1);
    nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.063); // 1 / max acceleration
    nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.1);    // 1 / max acceleration
    nh_private_.param<int>("posehistory_window", posehistory_window_, 1000);
    nh_private_.param<double>("trajectory_max_vel_x", trajectory_max_vel_x_, 15.0);
    nh_private_.param<double>("trajectory_max_vel_y", trajectory_max_vel_y_, 15.0);
    nh_private_.param<double>("trajectory_max_vel_z", trajectory_max_vel_z_, 10.0);
    nh_private_.param<double>("trajectory_max_acc_x", trajectory_max_acc_x_, 5.0);
    nh_private_.param<double>("trajectory_max_acc_y", trajectory_max_acc_y_, 5.0);
    nh_private_.param<double>("trajectory_max_acc_z", trajectory_max_acc_z_, 5.0);
    nh_private_.param<double>("trajectory_max_jerk_x", trajectory_max_jerk_x_, 5.0);
    nh_private_.param<double>("trajectory_max_jerk_y", trajectory_max_jerk_y_, 5.0);
    nh_private_.param<double>("trajectory_max_jerk_z", trajectory_max_jerk_z_, 5.0);
    nh_private_.param<double>("Kp_y", Kpos_y_, 5.0);
    nh_private_.param<double>("Kp_z", Kpos_z_, 5.0);
    nh_private_.param<double>("Kv_x", Kvel_x_, 1.5);
    nh_private_.param<double>("Kv_y", Kvel_y_, 1.5);
    nh_private_.param<double>("Kv_z", Kvel_z_, 3.3);

    targetPos_ << 0, 0, initTargetPos_z_; // Initial Position
    targetVel_ << 0.0, 0.0, 0.0;
    mavPos_ << 0.0, 0.0, 0.0;
    mavVel_ << 0.0, 0.0, 0.0;
    mavAtt_ << 1.0, 0.0, 0.0, 0.0;
    g_ << 0.0, 0.0, -9.807;
    drag_ << dx_, dy_, dz_;
    Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
    Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
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

    ruckigInput_.current_position = {mavPos_(0), mavPos_(1), mavPos_(2)};
    ruckigInput_.current_velocity = {mavVel_(0), mavVel_(1), mavVel_(2)};
    ruckigInput_.current_acceleration = {0, 0, 0};
    ruckigInput_.target_position = {targetWayPointPos_(0), targetWayPointPos_(1), targetWayPointPos_(2)};
    ruckigInput_.target_velocity = {0.0, 0.0, 0.0};
    ruckigInput_.target_acceleration = {0.0, 0.0, 0.0};
    ruckigInput_.max_velocity = {trajectory_max_vel_x_, trajectory_max_vel_y_, trajectory_max_vel_z_};
    ruckigInput_.max_acceleration = {trajectory_max_acc_x_, trajectory_max_acc_y_, trajectory_max_acc_z_};
    ruckigInput_.max_jerk = {trajectory_max_jerk_x_, trajectory_max_jerk_y_, trajectory_max_jerk_z_};

    waypointArrived_ = false;

    pubRefTraj();
}

void UAVController::statusLoopCallback(const ros::TimerEvent &event)
{
    if (readyToTakeoff_ && (controllerState_ == WAITING_TAKEOFF_COMMAND || controllerState_ == TAKING_OFF))
    {
        controllerState_ = TAKING_OFF;
        mavArmCommand_.request.value = true;
        mavSetMode_.request.custom_mode = "OFFBOARD";
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
        desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_);
        computeBodyRateCmd(cmdBodyRate_, desired_acc);
        pubReferencePose(targetPos_, q_des);
        pubRateCommands(cmdBodyRate_, q_des);
        appendPoseHistory();
        pubPoseHistory();

        if (distance(targetPos_, mavPos_) < TARGET_REACH_THRESHOLD)
        {
            waypointArrived_ = true;
            controllerState_ = FLYING;
        }
        break;
    case FLYING:
        ruckigResult_ = ruckig_.update(ruckigInput_, ruckigOutput_);
        if (ruckigResult_ == Result::Working)
        {
            targetPos_ << ruckigOutput_.new_position[0], ruckigOutput_.new_position[1], ruckigOutput_.new_position[2];
            targetVel_ << ruckigOutput_.new_velocity[0], ruckigOutput_.new_velocity[1], ruckigOutput_.new_velocity[2];
            targetAcc_ << ruckigOutput_.new_acceleration[0], ruckigOutput_.new_acceleration[1], ruckigOutput_.new_acceleration[2];

            // ROS_INFO_STREAM("targetPos_: " << targetPos_);
            // ROS_INFO_STREAM("targetVel_: " << targetVel_);
            // ROS_INFO_STREAM("targetAcc_: " << targetAcc_);
            ruckigOutput_.pass_to_input(ruckigInput_);
        }
        else if (ruckigResult_ == Result::Finished)
        {
            if (distance(targetPos_, mavPos_) < TARGET_REACH_THRESHOLD)
            {
                waypointArrived_ = true;
            }
        }

        desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_);
        computeBodyRateCmd(cmdBodyRate_, desired_acc);
        pubReferencePose(targetPos_, q_des);
        pubRateCommands(cmdBodyRate_, q_des);
        appendPoseHistory();
        pubPoseHistory();
        break;

    case LANDING:
        break;
    }
}

bool UAVController::takeoffServiceCallback(uav_controller::TakeoffCommand::Request &req, uav_controller::TakeoffCommand::Response &res)
{
    if (controllerState_ != WAITING_TAKEOFF_COMMAND)
        res.response = false;
    
    initTargetPos_z_ = req.absoluteAltitude;
    readyToTakeoff_ = true;
    res.response = true;

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

Eigen::Vector3d UAVController::controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc)
{
    /// Compute BodyRate commands using differential flatness
    /// Controller based on Faessler 2017
    const Eigen::Vector3d a_ref = target_acc;
    if (velocity_yaw_)
    {
        if (target_vel(0) != 0.0 || target_vel(1) != 0.0)
        {
            targetYaw_ = atan2(mavVel_(1), mavVel_(0));
        }
    }

    const Eigen::Vector3d pos_error = mavPos_ - target_pos;
    const Eigen::Vector3d vel_error = mavVel_ - target_vel;

    // Position Controller
    const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);

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

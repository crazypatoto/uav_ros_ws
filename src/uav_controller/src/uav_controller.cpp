#include "uav_controller/uav_controller.h"

UAVController::UAVController(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
{
    // Initialize subscribers
    mavStateSub_ = nh_.subscribe("mavros/state", 1, &UAVController::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
    groundTruthSub_ = nh_.subscribe("ground_truth/state", 1, &UAVController::groundTruthCallback, this, ros::TransportHints().tcpNoDelay());

    // Initialize publishers
    companionStatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);
    bodyRatePub_ = bodyRatePub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1);

    // Initialize service clients
    armingClient_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    setModeClient_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // Initialize timers
    statusLoopTimer_ = nh_.createTimer(ros::Duration(1), &UAVController::statusLoopCallback, this);   // Define timer for constant loop rate
    controlLoopTimer_ = nh_.createTimer(ros::Duration(1), &UAVController::controlLoopCallback, this); // Define timer for constant loop rate
}

void UAVController::mavstateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    currentMavState_ = *msg;
}

void UAVController::groundTruthCallback(const nav_msgs::Odometry &msg)
{
    if (!homePoseReceived && nodeState == WAITING_HOME_POSE)
    {
        homePoseReceived = true;
        homePose_ = msg.pose.pose;
        ROS_INFO_STREAM("Home pose initialized to: " << homePose_);
    }
}

void UAVController::statusLoopCallback(const ros::TimerEvent &event)
{
    // Enable OffBoard mode and arm automatically
    if (nodeState == FLYING)
    {
        mavArmCommand_.request.value = true;
        mavSetMode_.request.custom_mode = "OFFBOARD";
        if (currentMavState_.mode != "OFFBOARD" && (ros::Time::now() - lastRequest_ > ros::Duration(5.0)))
        {
            if (setModeClient_.call(mavSetMode_) && mavSetMode_.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            lastRequest_ = ros::Time::now();
        }
        else
        {
            if (!currentMavState_.armed && (ros::Time::now() - lastRequest_ > ros::Duration(5.0)))
            {
                if (armingClient_.call(mavArmCommand_) && mavArmCommand_.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                lastRequest_ = ros::Time::now();
            }
        }
    }
    publishCompanionState();
}

void UAVController::controlLoopCallback(const ros::TimerEvent &event)
{
    switch (nodeState)
    {
    case WAITING_FCU_CONNECTION:
        // Wait for FCU connection
        waitForPredicate(&currentMavState_.connected, "Connecting to FCU...", 2);
        ROS_INFO("Connected to FCU.");
        nodeState = WAITING_HOME_POSE;
        companionState_ = MAV_STATE::MAV_STATE_STANDBY;
        break;
    case WAITING_HOME_POSE:
        waitForPredicate(&homePoseReceived, "Waiting for home position...", 2);
        ROS_INFO("Got pose! Drone ready to be armed and take off.");
        nodeState = FLYING;
        companionState_ = MAV_STATE::MAV_STATE_ACTIVE;
        break;
    case FLYING:
        break;
    case LANDING:
        break;
    case LANDED:
        break;
    }
}

void UAVController::publishCompanionState()
{
    mavros_msgs::CompanionProcessStatus msg;

    msg.header.stamp = ros::Time::now();
    msg.component = 196; // MAV_COMPONENT_ID_AVOIDANCE
    msg.state = (int)companionState_;

    companionStatusPub_.publish(msg);
}
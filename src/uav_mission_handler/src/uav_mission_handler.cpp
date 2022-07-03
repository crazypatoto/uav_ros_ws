#include "uav_mission_handler/uav_mission_handler.h"

using namespace std;

UAVMissionHandler::UAVMissionHandler(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
{
    // Initialize subscribers
    controllerStateSub_ = nh_.subscribe("uav_controller/state", 1, &UAVMissionHandler::controllerStateCallback, this, ros::TransportHints().tcpNoDelay());
    uavPoseSub_ = nh_.subscribe("uav_controller/pose", 1, &UAVMissionHandler::uavPoseCallback, this, ros::TransportHints().tcpNoDelay());

    // Initialize publishers
    waypointPub_ = nh_.advertise<geometry_msgs::PoseStamped>("uav_controller/target_waypoint", 1);
    missionCompletePub_ = nh_.advertise<uav_msgs::Mission>("/mission_dispatcher/completed/mission", 10);
    orderCompletePub_ = nh_.advertise<uav_msgs::Order>("/mission_dispatcher/completed/order", 10);

    // Initialize service clients
    takeoffClient_ = nh_.serviceClient<uav_msgs::Takeoff>("uav_controller/takeoff");
    landClient_ = nh_.serviceClient<uav_msgs::Land>("uav_controller/land");
    goHomeClient_ = nh_.serviceClient<uav_msgs::GoHome>("uav_controller/go_home");
    spawnModelClient_ = nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    linkAttachClient_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    linkDetachClient_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

    // Initialize service servers
    orderReceiveServer_ = nh_.advertiseService("uav_mission_handler/enqueue_order", &UAVMissionHandler::orderReceivedCallback, this);

    // Initialize parameters
    nh_private_.param<string>("package_model_path", package_model_path_, ros::package::getPath("uav_gazebo") + "/models/package/model.sdf");
    nh_private_.param<int>("UAV_ID", uavID_, 0);
    nh_private_.param<double>("travel_altitude", travelAltitude_, 10);

    if (!filesystem::exists(package_model_path_))
    {
        ROS_INFO_STREAM("Cannot load model at: " << package_model_path_);
        ros::shutdown();
    }

    std::string ns = ros::this_node::getNamespace();
    ns.erase(remove(ns.begin(), ns.end(), '/'), ns.end()); // remove / from string
    packageNamePrefix_ = "package_";
    currentOrder_.finished = true;

    // Initialize timers
    missionLoopTimer_ = nh_.createTimer(ros::Duration(0.1), &UAVMissionHandler::missionLoopCallback, this); // Define timer for constant loop rate
}

void UAVMissionHandler::controllerStateCallback(const uav_msgs::State::ConstPtr &msg)
{
    uavState_ = *msg;
}

void UAVMissionHandler::uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uavPose_ = msg->pose;
}

void UAVMissionHandler::missionLoopCallback(const ros::TimerEvent &event)
{
    if (isDelivering_)
    {
        if (currentOrder_.finished) // UAV going home
        {
            if (uavState_.state == uav_msgs::State::WAITING_TAKEOFF_COMMAND)
            {
                ROS_INFO("UAV landed, ready for new order");
                isDelivering_ = false;
            }
            else if (uavState_.waypointArrived && uavState_.state != uav_msgs::State::LANDING)
            {
                ROS_INFO("Arrived home, landing...");
                land();
            }
        }
        else
        {
            if (uavState_.currentWaypoint.position.x == currentOrder_.missions[currentMissionIndex_].destination.x &&
                uavState_.currentWaypoint.position.y == currentOrder_.missions[currentMissionIndex_].destination.y &&
                uavState_.waypointArrived)
            {
                ROS_INFO("Arrived at mission #%d destination, dropping package...", currentMissionIndex_ + 1);
                detachPackage(currentOrder_.missions[currentMissionIndex_].packageID);
                currentOrder_.missions[currentMissionIndex_].finished = true;
                publishFinishedMission(currentOrder_.missions[currentMissionIndex_]);
                currentMissionIndex_++;

                if (currentMissionIndex_ >= currentOrder_.missions.size())
                {
                    ROS_INFO("All mission finished, going home...");
                    currentOrder_.finished = true;
                    publishFinishedOrder(currentOrder_);
                    goHome();
                }
                else
                {
                    ROS_INFO("Executing mission #%d...", currentMissionIndex_ + 1);
                    goTo(currentOrder_.missions[currentMissionIndex_].destination);
                }
            }
        }
    }
    else
    {
        if (currentOrder_.finished && uavState_.state == uav_msgs::State::WAITING_TAKEOFF_COMMAND)
        {
            if (orderQueue_.empty())
                return;

            ROS_INFO("Starting new order...");
            startNewOrder();
            takeoff();
        }
        else
        {
            if (uavState_.state == uav_msgs::State::FLYING)
            {
                ROS_INFO("Executing mission #1...");
                goTo(currentOrder_.missions[currentMissionIndex_].destination);
                isDelivering_ = true;
            }
        }
    }
}

bool UAVMissionHandler::orderReceivedCallback(uav_msgs::EnqueueOrder::Request &req, uav_msgs::EnqueueOrder::Response &res)
{
    orderQueue_.push(req.order);
    return true;
}

void UAVMissionHandler::spawnPackageModel(int packageID, int position)
{
    std::ifstream stream(package_model_path_);
    std::string xmlStr((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());

    gazebo_msgs::SpawnModel spawnModel;
    spawnModel.request.initial_pose.orientation.w = 0.707;
    spawnModel.request.initial_pose.orientation.x = 0;
    spawnModel.request.initial_pose.orientation.y = 0;
    spawnModel.request.initial_pose.orientation.z = 0.707;
    spawnModel.request.initial_pose.position.x = uavPose_.position.x;
    spawnModel.request.initial_pose.position.y = uavPose_.position.y + (0.14 * position);
    spawnModel.request.initial_pose.position.z = uavPose_.position.z - 0.1;

    ROS_INFO("Spawn at %.2f, %.2f, %.2f", spawnModel.request.initial_pose.position.x, spawnModel.request.initial_pose.position.y, spawnModel.request.initial_pose.position.z);
    spawnModel.request.model_name = packageNamePrefix_ + std::to_string(packageID);
    spawnModel.request.model_xml = xmlStr;

    if (spawnModelClient_.call(spawnModel) && spawnModel.response.success)
    {
        ROS_INFO("Model spawn success");
    }
    else
    {
        ROS_ERROR("Error spawning package model!");
    }
}

void UAVMissionHandler::attachPackage(int packageID)
{
    gazebo_ros_link_attacher::Attach attachCmd;
    attachCmd.request.model_name_1 = "iris" + std::to_string(uavID_);
    attachCmd.request.link_name_1 = "base_link";
    attachCmd.request.model_name_2 = packageNamePrefix_ + std::to_string(packageID);
    attachCmd.request.link_name_2 = "link";
    linkAttachClient_.call(attachCmd);

    if (!attachCmd.response.ok)
    {
        ROS_ERROR("Failed attaching package");
    }
}
void UAVMissionHandler::detachPackage(int packageID)
{
    gazebo_ros_link_attacher::Attach detachCmd;
    detachCmd.request.model_name_1 = "iris" + std::to_string(uavID_);
    detachCmd.request.link_name_1 = "base_link";
    detachCmd.request.model_name_2 = packageNamePrefix_ + std::to_string(packageID);
    detachCmd.request.link_name_2 = "link";

    if (!linkDetachClient_.call(detachCmd) || !detachCmd.response.ok)
    {
        ROS_ERROR("Failed detaching package");
    }
}

void UAVMissionHandler::startNewOrder()
{
    currentOrder_ = orderQueue_.front();
    orderQueue_.pop();
    currentMissionIndex_ = 0;

    switch (currentOrder_.missions.size())
    {
    case 2:
        spawnPackageModel(currentOrder_.missions[0].packageID, -1);
        spawnPackageModel(currentOrder_.missions[1].packageID, 1);
        break;
    case 3:
        spawnPackageModel(currentOrder_.missions[1].packageID, -1);
        spawnPackageModel(currentOrder_.missions[2].packageID, 1);
    case 1:
        spawnPackageModel(currentOrder_.missions[0].packageID, 0);
        break;
    default:
        ROS_ERROR("Unsupported mission size, order skipped!");
        currentOrder_.finished = true;
        break;
    }

    // Sleep for a while wait for package to drop on ground
    ros::Duration(0.5).sleep();

    for (int i = 0; i < currentOrder_.missions.size(); i++)
    {
        attachPackage(currentOrder_.missions[i].packageID);
    }
}

void UAVMissionHandler::takeoff()
{
    uav_msgs::Takeoff takeoffCmd;
    takeoffCmd.request.absoluteAltitude = travelAltitude_;

    if (!takeoffClient_.call(takeoffCmd) || !takeoffCmd.response.success)
    {
        ROS_ERROR("Failed taking off");
    }
}

void UAVMissionHandler::land()
{
    uav_msgs::Land landCmd;

    if (!landClient_.call(landCmd) || !landCmd.response.success)
    {
        ROS_ERROR("Failed landing");
    }
}

void UAVMissionHandler::goHome()
{
    uav_msgs::GoHome goHomeCmd;

    if (!goHomeClient_.call(goHomeCmd) || !goHomeCmd.response.success)
    {
        ROS_ERROR("Failed going home");
    }
}

void UAVMissionHandler::goTo(geometry_msgs::Vector3 destination)
{
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = destination.x;
    msg.pose.position.y = destination.y;
    msg.pose.position.z = travelAltitude_;

    waypointPub_.publish(msg);
}

void UAVMissionHandler::publishFinishedMission(uav_msgs::Mission &mission)
{
    missionCompletePub_.publish(mission);
}

void UAVMissionHandler::publishFinishedOrder(uav_msgs::Order &order)
{
    orderCompletePub_.publish(order);
}
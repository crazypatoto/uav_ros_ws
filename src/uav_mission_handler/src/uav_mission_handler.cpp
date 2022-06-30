#include "uav_mission_handler/uav_mission_handler.h"

using namespace std;

UAVMissionHandler::UAVMissionHandler(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private): nh_(nh), nh_private_(nh_private)
{
}

UAVMissionHandler::~UAVMissionHandler()
{
}

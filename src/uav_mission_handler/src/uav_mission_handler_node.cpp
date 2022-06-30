#include "uav_mission_handler/uav_mission_handler.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav_mission_handler");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  UAVMissionHandler *uavMissionHandler = new UAVMissionHandler(nh, nh_private);
  ros::spin();
  return 0;
}

#include "uav_controller/uav_controller.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  UAVController* uavController = new UAVController(nh, nh_private);
  ros::spin();
  return 0;
}

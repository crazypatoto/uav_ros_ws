#include "uav_controller/uav_controller.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  UAVController *uavController = new UAVController(nh, nh_private);
  

  dynamic_reconfigure::Server<uav_controller::UAVControllerConfig> srv;
  dynamic_reconfigure::Server<uav_controller::UAVControllerConfig>::CallbackType f;
  f = boost::bind(&UAVController::dynamicReconfigureCallback, uavController, _1, _2);
  srv.setCallback(f);

  ros::spin();
  return 0;
}

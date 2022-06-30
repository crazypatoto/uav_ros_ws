std_msgs/Header header

uint8 state
uint8 WAITING_FCU_CONNECTION = 0
uint8 WAITING_HOME_POSE = 1
uint8 WAITING_TAKEOFF_COMMAND = 2
uint8 TAKING_OFF = 3
uint8 FLYING = 4
uint8 LANDING = 5

geometry_msgs/Pose currentWaypoint
bool waypointArrived
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPose.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

bool home_pose_recived = false;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped global_setpoint;
geometry_msgs::PoseStamped local_setpoint;
geographic_msgs::GeoPose global_home;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
}

void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    if (home_pose_recived)
        return;
    if (msg->status.status != 0)
        return;
    global_home.position.altitude = msg->altitude;
    global_home.position.latitude = msg->latitude;
    global_home.position.longitude = msg->longitude;    
    home_pose_recived = true;
    ROS_INFO("Home Pose Set!: %f, %f, %f", global_home.position.latitude, global_home.position.longitude, global_home.position.altitude);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_uav_node");

    ros::NodeHandle nh;
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, global_pos_cb);

    ros::Publisher local_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher local_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Publisher global_setpoint_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    const char *ns = nh.getNamespace().c_str();

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    std::string world_frame = "world";
    std::string current_frame = nh.getNamespace() + "_world";
    current_frame.erase(remove(current_frame.begin(), current_frame.end(), '/'), current_frame.end()); // remove / from string

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("%s: Connected to FCU", ns);

    while (ros::ok() && !home_pose_recived)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geographic_msgs::GeoPoseStamped init_set_point;
    ROS_INFO("Seting %f, %f", global_home.position.latitude, global_home.position.longitude);
    init_set_point.pose.position.latitude = global_home.position.latitude;
    init_set_point.pose.position.longitude = global_home.position.longitude;
    init_set_point.pose.position.altitude = 19;
    init_set_point.pose.orientation = current_pose.pose.orientation;

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        global_setpoint_pub.publish(init_set_point);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {

        if (home_pose_recived)
        {
            if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(1.0)))
            {
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("%s: Offboard enabled", ns);
                }
                last_request = ros::Time::now();
            }
            else
            {
                if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(1.0)))
                {
                    if (arming_client.call(arm_cmd) &&
                        arm_cmd.response.success)
                    {
                        ROS_INFO("%s: Vehicle armed", ns);
                    }
                    last_request = ros::Time::now();
                }
            }
        }

        global_setpoint_pub.publish(init_set_point);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

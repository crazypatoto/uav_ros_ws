#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



bool debug = false;
bool start_receiving_setpoint = false;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped global_setpoint;
geometry_msgs::PoseStamped local_setpoint;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
}

void global_setpoint_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    global_setpoint = *msg;
    start_receiving_setpoint = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_uav_node");

    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Subscriber global_set_point_sub = nh.subscribe<geometry_msgs::PoseStamped>("global_position/setpoint", 10, global_setpoint_cb);
    ros::Publisher global_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("global_position/pose", 10);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    const char *ns = nh.getNamespace().c_str();

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    if (nh.getParam("debug", debug))
    {
        if (debug)
        {
            ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
            ROS_INFO("Set to Debug Mode.");
        }
    }

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

    geometry_msgs::PoseStamped init_set_point;
    init_set_point.pose.position.x = 0;
    init_set_point.pose.position.y = 0;
    init_set_point.pose.position.z = 1.5;

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_setpoint_pub.publish(init_set_point);
        ros::spinOnce();
        rate.sleep();
    }
    local_setpoint = init_set_point;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
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

        // Move UAV to target position
        if (!start_receiving_setpoint)
        {
            local_setpoint_pub.publish(local_setpoint);
        }
        else
        {
            try
            {
                geometry_msgs::TransformStamped tf = tfBuffer.lookupTransform(current_frame, world_frame, ros::Time(0));                ;
                tf2::doTransform(global_setpoint, local_setpoint, tf);
                local_setpoint_pub.publish(local_setpoint);
            }
            catch (tf2::TransformException ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }

        // Public global world pose
        try
        {
            geometry_msgs::TransformStamped tf = tfBuffer.lookupTransform(world_frame, current_frame, ros::Time(0));
            geometry_msgs::PoseStamped global_pose;
            tf2::doTransform(current_pose, global_pose, tf);
            global_pose_pub.publish(global_pose);
        }
        catch (tf2::TransformException ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

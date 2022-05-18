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

#define Z_OFFSET (0.5)

bool debug = false;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped current_set_point;
geometry_msgs::TransformStamped uav_world_tf;
int start_flag = 0;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
}

void global_set_point_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_set_point = *msg;
    start_flag = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_uav_node");

    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Subscriber global_set_point_sub = nh.subscribe<geometry_msgs::PoseStamped>("global_position/set_point", 10, global_set_point_cb);
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

    std::string source_frame = nh.getNamespace() + "_world";
    source_frame.erase(remove(source_frame.begin(), source_frame.end(), '/'), source_frame.end()); // remove / from string
    std::string target_frame = "world";

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("%s: Connected to FCU", ns);

    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position.x = 0;
    target_pose.pose.position.y = 0;
    target_pose.pose.position.z = Z_OFFSET;
    current_pose = target_pose;

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(target_pose);
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

        if (start_flag)
        {
            target_pose.pose.position.x = current_set_point.pose.position.x;
            target_pose.pose.position.y = current_set_point.pose.position.y;
            target_pose.pose.position.z = Z_OFFSET + current_set_point.pose.position.z;
        }


        // Move UAV to target position
        local_pos_pub.publish(target_pose);

        // Public global world pose
        try
        {
            uav_world_tf = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0));
            geometry_msgs::PoseStamped global_pose;
            tf2::doTransform(current_pose, global_pose, uav_world_tf);
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

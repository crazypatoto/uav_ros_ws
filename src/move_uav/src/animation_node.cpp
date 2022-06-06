/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <math.h>

geometry_msgs::PoseStamped uav0_current_pose;
geometry_msgs::PoseStamped uav1_current_pose;
geometry_msgs::PoseStamped uav2_current_pose;
geometry_msgs::PoseStamped uav3_current_pose;
geometry_msgs::PoseStamped uav4_current_pose;

double getDistance(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2)
{
    double x1 = pose1.pose.position.x;
    double y1 = pose1.pose.position.y;
    double z1 = pose1.pose.position.z;
    double x2 = pose2.pose.position.x;
    double y2 = pose2.pose.position.y;
    double z2 = pose2.pose.position.z;
    double d = pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2);
    d = sqrt(d);
    ROS_INFO("%.2f", d);
    return d;
}

void uav0_global_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav0_current_pose = *msg;
}
void uav1_global_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav1_current_pose = *msg;
}
void uav2_global_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav2_current_pose = *msg;
}
void uav3_global_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav3_current_pose = *msg;
}
void uav4_global_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav4_current_pose = *msg;
}

#define MAX_STEP 8

int uav0_x_setpoints[MAX_STEP] = {02, 02, 02, 02, 02, 02, 02, 02};
int uav0_y_setpoints[MAX_STEP] = {00, 00, 00, 00, 00, 00, 00, 00};
int uav0_z_setpoints[MAX_STEP] = {01, 01, 01, 01, 01, 01, 01, 01};

int uav1_x_setpoints[MAX_STEP] = {02, 01, 00, -1, -1, -1, -1, -1};
int uav1_y_setpoints[MAX_STEP] = {01, 01, 01, 01, 01, 01, 02, 02};
int uav1_z_setpoints[MAX_STEP] = {01, 01, 01, 01, 02, 03, 03, 02};

int uav2_x_setpoints[MAX_STEP] = {02, 02, 01, 00, -1, -1, -1, -1};
int uav2_y_setpoints[MAX_STEP] = {02, 01, 01, 01, 01, 01, 01, 02};
int uav2_z_setpoints[MAX_STEP] = {01, 01, 01, 01, 01, 02, 03, 03};

int uav3_x_setpoints[MAX_STEP] = {02, 02, 02, 01, 00, -1, -1, -1};
int uav3_y_setpoints[MAX_STEP] = {03, 02, 01, 01, 01, 01, 01, 01};
int uav3_z_setpoints[MAX_STEP] = {01, 01, 01, 01, 01, 01, 02, 03};

int uav4_x_setpoints[MAX_STEP] = {02, 02, 02, 02, 01, 00, -1, -1};
int uav4_y_setpoints[MAX_STEP] = {04, 03, 02, 01, 01, 01, 01, 01};
int uav4_z_setpoints[MAX_STEP] = {01, 01, 01, 01, 01, 01, 01, 02};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "animation_nodekeyboard_move_node");
    ros::NodeHandle nh;

    ros::Publisher uav0_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav0/global_position/setpoint", 10);
    ros::Subscriber uav0_global_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav0/global_position/pose", 10, uav0_global_pose_cb);
    ros::Publisher uav1_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav1/global_position/setpoint", 10);
    ros::Subscriber uav1_global_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/global_position/pose", 10, uav1_global_pose_cb);
    ros::Publisher uav2_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav2/global_position/setpoint", 10);
    ros::Subscriber uav2_global_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav2/global_position/pose", 10, uav2_global_pose_cb);
    ros::Publisher uav3_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav3/global_position/setpoint", 10);
    ros::Subscriber uav3_global_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav3/global_position/pose", 10, uav3_global_pose_cb);
    ros::Publisher uav4_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav4/global_position/setpoint", 10);
    ros::Subscriber uav4_global_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav4/global_position/pose", 10, uav4_global_pose_cb);
    geometry_msgs::PoseStamped uav0_setpoint, uav1_setpoint, uav2_setpoint, uav3_setpoint, uav4_setpoint;

    ros::Rate rate(10.0);
    int index = 0;

    uav0_setpoint.pose.position.x = uav0_x_setpoints[index] + 0.5;
    uav0_setpoint.pose.position.y = uav0_y_setpoints[index] + 0.5;
    uav0_setpoint.pose.position.z = uav0_z_setpoints[index] + 0.5;
    uav1_setpoint.pose.position.x = uav1_x_setpoints[index] + 0.5;
    uav1_setpoint.pose.position.y = uav1_y_setpoints[index] + 0.5;
    uav1_setpoint.pose.position.z = uav1_z_setpoints[index] + 0.5;
    uav2_setpoint.pose.position.x = uav2_x_setpoints[index] + 0.5;
    uav2_setpoint.pose.position.y = uav2_y_setpoints[index] + 0.5;
    uav2_setpoint.pose.position.z = uav2_z_setpoints[index] + 0.5;
    uav3_setpoint.pose.position.x = uav3_x_setpoints[index] + 0.5;
    uav3_setpoint.pose.position.y = uav3_y_setpoints[index] + 0.5;
    uav3_setpoint.pose.position.z = uav3_z_setpoints[index] + 0.5;
    uav4_setpoint.pose.position.x = uav4_x_setpoints[index] + 0.5;
    uav4_setpoint.pose.position.y = uav4_y_setpoints[index] + 0.5;
    uav4_setpoint.pose.position.z = uav4_z_setpoints[index] + 0.5;

    while (ros::ok())
    {

        int flag = 1;
        if (getDistance(uav0_current_pose, uav0_setpoint) > 0.5)
        {
            flag &= 0;
        }
        if (getDistance(uav1_current_pose, uav1_setpoint) > 0.5)
        {
            flag &= 0;
        }
        if (getDistance(uav2_current_pose, uav2_setpoint) > 0.5)
        {
            flag &= 0;
        }
        if (getDistance(uav3_current_pose, uav3_setpoint) > 0.5)
        {
            flag &= 0;
        }
        if (getDistance(uav4_current_pose, uav4_setpoint) > 0.5)
        {
            flag &= 0;
        }

        if (flag == 1 && (index + 1) < MAX_STEP)
        {
            index++;
            uav0_setpoint.pose.position.x = uav0_x_setpoints[index] + 0.5;
            uav0_setpoint.pose.position.y = uav0_y_setpoints[index] + 0.5;
            uav0_setpoint.pose.position.z = uav0_z_setpoints[index] + 0.5;
            uav1_setpoint.pose.position.x = uav1_x_setpoints[index] + 0.5;
            uav1_setpoint.pose.position.y = uav1_y_setpoints[index] + 0.5;
            uav1_setpoint.pose.position.z = uav1_z_setpoints[index] + 0.5;
            uav2_setpoint.pose.position.x = uav2_x_setpoints[index] + 0.5;
            uav2_setpoint.pose.position.y = uav2_y_setpoints[index] + 0.5;
            uav2_setpoint.pose.position.z = uav2_z_setpoints[index] + 0.5;
            uav3_setpoint.pose.position.x = uav3_x_setpoints[index] + 0.5;
            uav3_setpoint.pose.position.y = uav3_y_setpoints[index] + 0.5;
            uav3_setpoint.pose.position.z = uav3_z_setpoints[index] + 0.5;
            uav4_setpoint.pose.position.x = uav4_x_setpoints[index] + 0.5;
            uav4_setpoint.pose.position.y = uav4_y_setpoints[index] + 0.5;
            uav4_setpoint.pose.position.z = uav4_z_setpoints[index] + 0.5;
            uav0_setpoint_pub.publish(uav0_setpoint);
            ros::Duration(1.0).sleep();
            uav1_setpoint_pub.publish(uav1_setpoint);
            ros::Duration(1.0).sleep();
            uav2_setpoint_pub.publish(uav2_setpoint);
            ros::Duration(1.0).sleep();
            uav3_setpoint_pub.publish(uav3_setpoint);
            ros::Duration(1.0).sleep();
            uav4_setpoint_pub.publish(uav4_setpoint);
            ros::Duration(1.0).sleep();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

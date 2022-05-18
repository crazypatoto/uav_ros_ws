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

char key;

// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_move_node");
    ros::NodeHandle nh;
    
    ros::Publisher set_point_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("global_position/set_point", 10);

    ros::Rate rate(100.0);  

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;
    
    set_point_pub.publish(pose);    // Public set point once
  
    while(ros::ok()){
        key = getch();
        ROS_INFO("You just pressd %c", key);
        
        switch (key)
        {
        case 'w':
        case 'W':
              pose.pose.position.x += 1;
            break;
        case 's':
        case 'S':
            pose.pose.position.x -= 1;
            break;
        case 'a':
        case 'A':
            pose.pose.position.y += 1;
            break;
        case 'd':        
        case 'D':        
            pose.pose.position.y -= 1;
            break;
        case 'q':        
        case 'Q':        
            break;
        case 'e':        
        case 'E':        
            break;
        case 'r':        
        case 'R':        
            pose.pose.position.z += 1;
            break;
        case 'f':        
        case 'F':        
            pose.pose.position.z -= 1;
            break;
        case 27:
            ROS_INFO("Leaving...");        
            return 0;
        default:
            break;
        }
        
        ROS_INFO("x: %.0f, y: %.0f, z: %.0f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

        set_point_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


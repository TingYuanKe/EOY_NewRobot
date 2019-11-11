#ifndef __CONTROL_API_H__
#define __CONTROL_API_H__

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"

#define MAX_LIN_VEL 0.26
#define MAX_ANG_VEL 1.82

#define LIN_VEL_STEP_SIZE 0.01
#define ANG_VEL_STEP_SIZE 0.1

ros::Publisher pub;

geometry_msgs::Twist cmd_vel;

void rosInit(int argc, char **argv, const char *node_name)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    return;
}

void setSpeed(int left_speed, int right_speed)
{
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    
    pub.publish(cmd_vel);
    
    ros::spinOnce();
    return;
}


void goForward()
{
    cmd_vel.linear.x = MAX_LIN_VEL;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    
    pub.publish(cmd_vel);

    ros::spinOnce();
    return;
}

void goBack()
{
    cmd_vel.linear.x = -MAX_LIN_VEL;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    
    pub.publish(cmd_vel);

    ros::spinOnce();
    return;
}

void spinLeft()
{
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.7;
    
    pub.publish(cmd_vel);
    ros::spinOnce();
    return;
}

void spinRight()
{
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = -0.7;
    
    pub.publish(cmd_vel);
    ros::spinOnce();
    return;
}

void turnLeftRight(bool isleft)
{
    cmd_vel.linear.x = MAX_LIN_VEL;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = isleft ? 0.7 : -0.7;
    
    pub.publish(cmd_vel);
    ros::spinOnce();
    return;
}

void stopMotion()
{
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    
    pub.publish(cmd_vel);
    ros::spinOnce();
    return;
}

void resetSpeed()
{
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    
    pub.publish(cmd_vel);

    ros::spinOnce();
    return;
}

#endif

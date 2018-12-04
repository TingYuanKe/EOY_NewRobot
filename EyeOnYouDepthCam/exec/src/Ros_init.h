#ifndef __ROS_INIT_H__
#define __ROS_INIT_H__

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
ros::Publisher pub5;
ros::Publisher pub6;

ros::Publisher pub7;
ros::Publisher pub8;

ros::Publisher pub9;
ros::Publisher pub10;
ros::Publisher pub11;

ros::Publisher pub12;
ros::Publisher pub13;


void rosInit(int argc, char **argv, const char *node_name)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    pub1 = n.advertise<std_msgs::Int32MultiArray>("robot_speed", 100);
    pub2 = n.advertise<std_msgs::Int32MultiArray>("leg_speed", 100);
    pub3 = n.advertise<std_msgs::Int32MultiArray>("robot_MA", 100);
    pub4 = n.advertise<std_msgs::Int32MultiArray>("leg_MA", 100);
    pub5 = n.advertise<std_msgs::Int32MultiArray>("robot_HO", 100);
    pub6 = n.advertise<std_msgs::Int32MultiArray>("robot_VA", 100);

    pub7 = n.advertise<std_msgs::Int32>("robot_motion", 100);
    pub8 = n.advertise<std_msgs::Int32>("leg_motion", 100);

    pub9 = n.advertise<std_msgs::Int32MultiArray>("leg_HO", 100);
    pub10 = n.advertise<std_msgs::Int32MultiArray>("leg_VA", 100);
    pub11 = n.advertise<std_msgs::Int32MultiArray>("robot_MR", 100);

    pub12 = n.advertise<std_msgs::Float64>("head_top", 100);
    pub13 = n.advertise<std_msgs::Float64>("head_bottom", 100);

    return;
}

#endif

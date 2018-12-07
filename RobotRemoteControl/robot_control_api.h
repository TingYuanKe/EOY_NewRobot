#ifndef __CONTROL_API_H__
#define __CONTROL_API_H__

#include "ros/ros.h"

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

std_msgs::Int32MultiArray robot_speed,leg_speed,robot_MA,leg_MA,robot_HO,robot_VA,robot_MR,leg_HO,leg_VA;
std_msgs::Int32 robot_motion,leg_motion;
std_msgs::Float64 head_top,head_bottom;

int vv1 = 1000, vv2 = 1000;

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

void setSpeed(int left_speed, int right_speed)
{
    robot_speed.data.clear();

    robot_speed.data.push_back(left_speed);
    robot_speed.data.push_back(right_speed);
    pub1.publish(robot_speed);
    ros::spinOnce();
    return;
}

void setAcceleration(int left_acc, int right_acc)
{
    robot_VA.data.clear();

    robot_VA.data.push_back(left_acc);
    robot_VA.data.push_back(right_acc);
    pub6.publish(robot_VA);
    ros::spinOnce();
    return;
}

void goForward()
{
    robot_motion.data = 1;
    pub7.publish(robot_motion);
    ros::spinOnce();
    return;
}

void goBack()
{
    robot_motion.data = 2;
    pub7.publish(robot_motion);
    ros::spinOnce();
    return;
}

void spinLeft()
{
    robot_motion.data = 3;
    pub7.publish(robot_motion);
    ros::spinOnce();
    return;
}

void spinRight()
{
    robot_motion.data = 4;
    pub7.publish(robot_motion);
    ros::spinOnce();
    return;
}

void turnLeftRight(int left_speed, int right_speed)
{
    setSpeed(left_speed, right_speed);
    goForward();
    ros::spinOnce();
    return;
}

void stopMotion()
{
    robot_motion.data = 0;
    pub7.publish(robot_motion);
    ros::spinOnce();
    return;
}

void resetSpeed()
{
    robot_speed.data.clear();
    leg_speed.data.clear();

    robot_speed.data.push_back(350);
    robot_speed.data.push_back(350);
    leg_speed.data.push_back(100);
    leg_speed.data.push_back(100);
    pub1.publish(robot_motion);
    pub2.publish(leg_speed);
    vv1 = 350;
    vv2 = 350;

    ros::spinOnce();
    return;
}

void armUp()
{
    leg_MA.data.clear();

    leg_MA.data.push_back(-26000);
    leg_MA.data.push_back(-27000);
    pub4.publish(leg_MA);
    ros::spinOnce();
    return;
}

void armDown()
{
    leg_MA.data.clear();

    leg_MA.data.push_back(0);
    leg_MA.data.push_back(0);
    pub4.publish(leg_MA);
    ros::spinOnce();
}

#endif

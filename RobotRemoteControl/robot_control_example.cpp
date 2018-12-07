#include "ros/ros.h"
#include <cstdio>
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <iostream> //lib use in c++
#include "conio.h"

#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "robot_control_api.h"

int main(int argc, char **argv)
{
    // init the ros node
    rosInit(argc, argv, "Track_node");

    // set the frequency of the while loop to 100hz
    // avoid to publish the commands too frequently
    ros::Rate loop_rate(100);

    char y;
    while(ros::ok())
    {
        y = getch();

        if(y == 'h')
        {
            printf("==========================\n");
            printf("Key usage:\n");
            printf(" w: go front\n");
            printf(" a: spin left\n");
            printf(" s: go back\n");
            printf(" d: spin right\n");
            printf(" p(or space): stop\n");
            printf(" r: turn right\n");
            printf(" l: turn left\n");
            printf(" q: set acceleration\n");
            printf(" t: set speed\n");
            printf(" j: arm up\n");
            printf(" m: arm down\n");
            printf(" e: exit the program\n");
            printf("==========================\n");
        }

        if(y == 'e')
        {
            printf("exit\n");
            break;
        }

        if(y == 'w')
        {
            printf("forward\n");
            goForward();
        }

        if(y == 's')
        {
            printf("back\n");
            goBack();
        }

        if(y == 'a')
        {
            printf("left\n");
            spinLeft();
        }

        if(y == 'd')
        {
            printf("right\n");
            spinRight();
        }

        if(y == 'r')
        {
            printf("turn right\n");
            turnLeftRight(1500, 1000);
        }

        if(y == 'l')
        {
            printf("turn left\n");
            turnLeftRight(1000, 1500);
        }

        if(y == 'j')
        {
            printf("arm up\n");
            armUp();
        }

        if(y == 'm')
        {
            printf("arm down\n");
            armDown();
        }

        if(y == 'p' || y == ' ')
        {
            printf("stop\n");
            stopMotion();
        }

        if(y == 't')
        {
            int left_speed, right_speed;
            printf("set speed\n");
            printf("left: ");
            scanf("%d", &left_speed);
            printf("right: ");
            scanf("%d", &right_speed);

            if(left_speed < 300 || left_speed > 30000 || right_speed < 300 || right_speed > 30000)
                printf("speed must range in 300 ~ 30000\n");
            else
            {
                setSpeed(left_speed, right_speed);
                printf("set speed: %d %d\n", left_speed, right_speed);
            }
        }

        if(y == 'q')
        {
            int left_acc, right_acc;
            printf("set acceleration\n");
            printf("left: ");
            scanf("%d", &left_acc);
            printf("right: ");
            scanf("%d", &right_acc);

            setAcceleration(left_acc, right_acc);
        }
        // loop rate 100hz
        loop_rate.sleep();
    }
    return 0;
}

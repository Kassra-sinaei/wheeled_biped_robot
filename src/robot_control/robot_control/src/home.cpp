#include "ros/ros.h"
#include "ros/console.h"
#include <iostream>
#include "std_msgs/Float64.h"
#include "cmath"


int main(int argc, char **argv)
{
    ros::init(argc,argv,"home");
    ros::NodeHandle nh;

    ros::Publisher home_advert1 = nh.advertise<std_msgs::Float64>("/joint1_position_controller/command", 50);
    ros::Publisher home_advert2 = nh.advertise<std_msgs::Float64>("/joint2_position_controller/command", 50);
    ros::Publisher home_advert3 = nh.advertise<std_msgs::Float64>("/joint3_position_controller/command", 50);
    ros::Publisher home_advert4 = nh.advertise<std_msgs::Float64>("/joint4_position_controller/command", 50);
    ros::Publisher home_advert5 = nh.advertise<std_msgs::Float64>("/joint5_position_controller/command", 50);
    ros::Publisher home_advert6 = nh.advertise<std_msgs::Float64>("/joint6_position_controller/command", 50);
    while(ros::ok()){
        std_msgs::Float64 msg;
        msg.data = 0.0;
        home_advert1.publish(msg);
        home_advert2.publish(msg);
        msg.data = 0.2;
        home_advert3.publish(msg);
        msg.data = 0.0;
        home_advert4.publish(msg);
        home_advert5.publish(msg);
        msg.data = -0.2;
        home_advert6.publish(msg);
        ROS_INFO("Home Positions Published");
        ros::spinOnce();
    }
    

    return 0;
}
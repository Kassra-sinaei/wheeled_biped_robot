#include "robot.h"
#include "ros/ros.h"
#include "ros/console.h"
#include <iostream>
#include "cmath"
#include <math.h>
#include"Eigen/Dense"
#include "Eigen/eiquadprog.h"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include <Eigen/Geometry>
#include <vector>


int main(int argc, char **argv)
{
    ros::init(argc,argv,"home");
    ros::NodeHandle nh;
    //robot robot;

    while(ros::ok){
        ROS_INFO("Controller Spinning");
    }
    return 0;
}
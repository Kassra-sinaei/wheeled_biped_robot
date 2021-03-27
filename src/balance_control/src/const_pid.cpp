#include "robot.h"

#include "ros/ros.h"
#include "ros/console.h"
#include<sensor_msgs/Imu.h>

#include <iostream>
#include "cmath"
#include <math.h>
#include <vector>

#include"Eigen/Dense"
#include "Eigen/eiquadprog.h"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"controller");
    ros::NodeHandle nh;
    robot wb(0.2,0.2,0.08,0.22,0.05);

    while(ros::ok){
        ROS_INFO("Controller Spinning");
    }
    return 0;
}
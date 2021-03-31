#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "robot.h"

#include "ros/ros.h"
#include "ros/console.h"
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

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


class controller
{
    public:
        controller(robot *robot_pointer);

    private:
        ros::Subscriber row_imu;
        ros::Subscriber odom;
        ros::Publisher joint1_pub;
        ros::Publisher joint2_pub;
        ros::Publisher joint3_pub;
        ros::Publisher joint4_pub;
        ros::Publisher joint5_pub;
        ros::Publisher joint6_pub;

        ros::Publisher imu_y;

        robot *rbt;

        //PID Controller Variables
        double previous_error_vel;
        double previous_error_phi;
        double i_vel;
        double i_phi;

        void imu_rpy(const sensor_msgs::Imu::ConstPtr& msg);
        void set_vel(const nav_msgs::Odometry &msg);
        double vel_controller(double target_vel, double current_vel, double kp = 500, double ki = 0.01, double kd = 20.0);  
        float* leg_config(double height, double etha);
        double wheel_speed(double current_phi, double desired_phi = 0, double kp = 500, double ki = 0.01, double kd = 20.0);
};

#endif
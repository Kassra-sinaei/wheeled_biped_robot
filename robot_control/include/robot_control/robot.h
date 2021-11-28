#pragma once

#include <eigen3/Eigen/Eigen>

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"

#include <ros/ros.h>
#include <ros/console.h>
#include "robot_control/Joint_cmd.h"
#include "robot_control/Gain.h"

using namespace Eigen;
using namespace std;

class Robot{
    public:
        Robot(bool knee_mechanism, double hip, double shank, double n, double l, double r, double diameter, double dt);
    private:
        //Joint Lengths
        double hip_l;
        double shank_l;
        double r_l;
        double l_l;
        double n_l;
        double d_wheel;

        double current_height;
        MatrixXd current_gain1;
        MatrixXd current_gain2;
        Matrix2d decouple;

        double dt;
        double time;

        double* robot_pose(double eta, double h);
        double knee_linkage(double theta);
        double clamp(double value, double high, double low);
        bool spinOnline(robot_control::Joint_cmd::Request &req, robot_control::Joint_cmd::Response &res);
        double zero_eta(double height);
        double pend_length(double height);

        bool knee;
        fstream file;

        ros::NodeHandle nh;
        ros::ServiceServer joint_cmd;
        ros::ServiceClient lqr_gain;
};
#pragma once

#include <eigen3/Eigen/Eigen>

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"

#include <ros/ros.h>
#include <ros/console.h>
#include "robot_control/Gain.h"


using namespace Eigen;
using namespace std;

//double K_G = 9.81;

class LQR{
    public:
        LQR(double l, double M, double m, double j_w, double j_p, double r, double d);
        bool updateModel(double l);
        bool updateGain(ros::NodeHandle& nh);
    private:
        MatrixXd A;
        MatrixXd B;
        MatrixXd Q;
        MatrixXd R;
        MatrixXd K;
        double m, M, j_w, j_p, r, d;

        double* eigen2Array(MatrixXd mat, int m, int n);
};
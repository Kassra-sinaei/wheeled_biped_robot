#pragma once

#include <eigen3/Eigen/Eigen>

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"

#include <ros/ros.h>
#include <ros/console.h>

using namespace Eigen;
using namespace std;

class Jump{
    public:
        Jump(double time_step);
        double* generateTraj(double l_0, double l_d_0, double l_dd_0, double l_to, double t_0, double t_to, double t_recover, double zd, int &len);
        ~Jump();
    private:
        bool fitPoly5(double (&res)[6], double init_x, double init_xd, double init_xdd,double end_x, double end_xd, double end_xdd, double t);
        double CoM2Pelvis(double com);
        ofstream file;
        double dt;
};
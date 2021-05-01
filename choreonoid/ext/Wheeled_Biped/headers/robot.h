#pragma once

#include"Eigen/Dense"
#include "Eigen/eiquadprog.h"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include <Eigen/Geometry>

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"

#include "headers/controller.h"

using namespace Eigen;
using namespace std;


class ROBOT{
public:
    ROBOT(double hip, double shank, double n, double l, double r);
    vector<double> spin(Matrix3d gyro, Matrix3d accel);
private:
    //Joint Lengths
    double hip_l;
    double shank_l;
    double r_l;
    double l_l;
    double n_l;

    Vector3d dv;
    Vector3d base_R;

    CONTROLLER* ctrl;

    double* robot_pose(double eta, double h);
    double knee_linkage(double theta);
    void update_sensors(Vector3d gyro, Vector3d accelerometer);
    vector<double> send_motion_commands();
};
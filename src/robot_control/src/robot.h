#ifndef ROBOT_H
#define ROBOT_H

#include "ros/ros.h"
#include "ros/console.h"

#include <iostream>
#include "cmath"
#include <math.h>
#include <vector>

#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

#include"Eigen/Dense"
#include "Eigen/eiquadprog.h"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;


class robot{
public:
    robot(double h, double s, double n, double r, double l);
    void set_pose_attitude(Vector3d pose, VectorXd attitude);
    //void set_params(double h, double s, double n, double r, double l);
    double l_theta2sim(double theta1);
    double theta2psi(double theta2);
private:
    double H; // Hip Length
    double S; // Shank length
    double N; // Knee Mechanism Lengths
    double R; // Knee Mechanism Lengths
    double L; // Knee Mechanism Lengths

    Vector3d base_pose;
    Vector3d base_attitude;

    float q[6];

    Vector3d quaternion2euler(VectorXd quaternion);
    Vector3d quat2rot(VectorXd quaternion);
    Vector3d rot2rpy(MatrixXd rotation);
    
};

#endif //ROBOT_H
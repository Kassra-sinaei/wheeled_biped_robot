#ifndef ROBOT_H
#define ROBOT_H

#include "ros/ros.h"
#include "ros/console.h"
#include <iostream>
#include "std_msgs/Float64.h"
#include "cmath"
#include <math.h>
#include"Eigen/Dense"
#include "Eigen/eiquadprog.h"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include <Eigen/Geometry>
#include <vector>

using namespace Eigen;
using namespace std;


class robot{
public:
    robot();
private:
    Vector3d base_pose;
    Vector3d base_attitude;

    float q[6];
};

#endif 
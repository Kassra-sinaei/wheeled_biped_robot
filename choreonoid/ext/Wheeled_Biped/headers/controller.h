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

using namespace std;
using namespace Eigen;

class CONTROLLER{
    /*
        Methods required for creating a closed loop controller
        x2 PIDC: 1 - Balance Controller
                 2 - Base Velocity Controller
    */
    friend class ROBOT;

public:
    CONTROLLER();

private:
    double old_phi;     // Balance Controller previous phi error
    double i_phi;       // Balance Controller Integrator
    
    double old_eta;     // Speed Controller previous error
    double i_eta;       // Speed Controller Integrator

    double phi_control(double phi_d, double phi, double kp, double ki, double kd);
    double eta_control(double eta_d, double eta, double kp, double ki, double kd);

};
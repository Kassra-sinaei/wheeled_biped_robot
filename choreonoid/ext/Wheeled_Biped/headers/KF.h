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

class KF{
    public:
        KF(MatrixXd A, MatrixXd B, MatrixXd H, double Q, double R, MatrixXd X0, MatrixXd p_ini);
        MatrixXd getState();
        void estimate(MatrixXd U, MatrixXd Y);

    private:
        MatrixXd x_hat;     // Estimated State
        MatrixXd x_bar;
        MatrixXd P_bar;     // Variance Posterior
        MatrixXd P;         // Variance Prior
        MatrixXd A;         // state space A
        MatrixXd B;         // state space B
        MatrixXd H;         // state space H
        double r;           // measurement noise
        double q;           // process noise

        double* gauss_pdf();
        void predict(MatrixXd u);
        void update(MatrixXd z);
};
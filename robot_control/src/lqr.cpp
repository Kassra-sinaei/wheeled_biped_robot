#include "lqr.h"

LQR::LQR(double l, double M, double m, double j_w, double j_p, double r, double d){
    this->m = m;
    this->M = M;
    this->j_p = j_p;
    this->j_w = j_w;
    this->r = r;
    this->d = d;
    this->A = MatrixXd::Zero(6,6);
    this->B = MatrixXd::Zero(6,2);
    this->Q = MatrixXd::Zero(6,6);
    this->R = MatrixXd::Zero(2,2);
    updateModel(l);
}

bool LQR::updateModel(double l){
    double K_G = 9.81;
    double x = 2 * j_w * j_p + 2 * M * pow(l,2) * j_w + M * pow(r,2) * j_p +
                2 * m * pow(r,2) * j_p + 2 * m * M * pow(l * r, 2);
    //this->A(1,2) = -K_G * l * pow(M,2) * pow(r, 2)/ x;
    //this->A(3,2) = l * M * K_G * (2 * j_w + M * pow(r,2) + 2 * m * pow(r,2)) / x;
    //this->B(1,1) = r * (M * pow(l,2) + M * l * r + j_w) / x;
    //this->B(1,0) = this->B(1,1);
    //this->B(3,0) = -(2 * j_w + M * pow(r,2) + 2 * m * pow(r,2) + l * M * r) / x;
    //this->B(3,1) = this->B(3,0);
    //this->B(5,0) = d * r / (pow(d,2) * (m * pow(r,2) + j_w) + 2 * (M*pow(d,2)/12) * pow(r,2));
    //this->B(5,1) = -this->B(5,0);

    return true;
}

bool LQR::updateGain(ros::NodeHandle& nh){
    ros::ServiceClient client = nh.serviceClient<robot_control::Gain>("lqr_gain");
    robot_control::Gain message;

    float a[36];
    float b[12];
    float q[6];
    float r[2] = {0.0, 0.0};
    eigen2Array(this->A, 6, 6, a);
    eigen2Array(this->B, 6, 2, b);
    eigen2Array(this->Q, 6, 1, q);
    eigen2Array(this->R, 2, 1, r);
    //for(int i = 0; i < 36; i ++)
    //    message.request.A[i] = a[i];
    //for(int i = 0 ; i < 12; i ++)
    //    message.request.B[i] = b[i];
    //for(int i = 0 ; i < 6; i++)
    //    message.request.Q[i] = q[i];
    //for (int i = 0; i < 2; i ++)
    //    message.request.R[i] = r[i];
    return true;
}

void LQR::eigen2Array(MatrixXd mat, int m, int n, float* res){
    for(int i = 0; i < m; i ++)
        for(int j = 0 ; j < n; j ++)
            res[i * n + j] = mat(i,j);
}
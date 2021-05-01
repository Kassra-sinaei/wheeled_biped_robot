#include "headers/robot.h"

ROBOT::ROBOT(double hip, double shank, double n, double l, double r){
    this->hip_l = hip;
    this->shank_l = shank;
    this->n_l = n;
    this->l_l = l;
    this->r_l = r;

    ctrl = new CONTROLLER;

    cout << "Robot Object Initialized...\n";
}

vector<double> ROBOT::send_motion_commands(double speed, double height){
    double q0, q1, q2;

    //double eta = ctrl->eta_control();
    double eta = 0.0;

    double* config = this->robot_pose(eta, height);
    q0 = config[2] - 43.0;

    q1 = 127.0 - knee_linkage(config[1]);

    //q2 = ctrl->phi_control(0.0,base_R(1))
    q2 = 0;

    vector<double> ans{q0,q1,q2};
    return ans;
}

void ROBOT::update_sensors(Vector3d gyro, Vector3d accelerometer){
    this->base_R = gyro;
    this->dv = accelerometer;
}

double ROBOT::knee_linkage(double theta){
    // Analytic Solve of Knee 4-bar Mechanism
    double a = sqrt(hip_l*hip_l + n_l*n_l - 2*hip_l*hip_l*cos(theta));
    double psi1 = asin(n_l/a * sin(theta));
    double psi2 = acos((a*a + r_l*r_l - l_l*l_l) / (2*a*r_l));
    return psi1 + psi2;
}

double* ROBOT::robot_pose(double eta, double h){
    double *theta = new double[3];
    double t1,t2,t3,t5,t7,t8,t10,t19,t20,t24,t17,t16;

    t1 = tan(eta);
    t2 = t1 * t1;
    t3 = t2 * h;
    t5 = t2 * t2;
    t7 = h * h;
    t17 = sqrt(-0.25e2 * t7 * t5 * t2 - 0.25e2 * t2 * t7 - 0.50e2 * t5 * t7 + 0.4e1 * t2 + 0.4e1 * t5);
    t19 = 0.5e1 * t3 + t17 + 0.5e1 * h;
    t20 = t2 + 0.1e1;
    double theta1_1 = atan2(0.1e1 / t20 * t19 / 0.2e1, -0.5000000000e0 / t1 * (-0.5e1 * t3 - 0.5e1 * h + 0.1000000000e1 / t20 * t19));

    t1 = tan(eta);
    t2 = t1 * t1;
    t3 = t2 * h;
    t8 = t2 * t2;
    t10 = h * h;
    t16 = t2 * t10;
    t20 = sqrt(-0.25e2 * t10 * t8 * t2 - 0.50e2 * t8 * t10 - 0.25e2 * t16 + 0.4e1 * t2 + 0.4e1 * t8);
    t24 = 0.1e1 / t1;
    double theta2_1 = atan2(0.2500000000e0 / (t2 + 0.1e1) * t24 * (0.5e1 * t3 + t20 + 0.5e1 * h) * (-0.10e2 * t3 - 0.10e2 * h) + 0.5000000000e0 * t24 * (0.25e2 * t16 + 0.25e2 * t10), 0.1250000000e2 * t16 + 0.1250000000e2 * t10 - 0.1e1);

    t1 = tan(eta);
    t2 = t1 * t1;
    t3 = t2 * h;
    t5 = t2 * t2;
    t7 = h * h;
    t17 = sqrt(-0.25e2 * t7 * t5 * t2 - 0.25e2 * t2 * t7 - 0.50e2 * t5 * t7 + 0.4e1 * t2 + 0.4e1 * t5);
    t19 = -0.5e1 * t3 + t17 - 0.5e1 * h;
    t20 = t2 + 0.1e1;
    double theta1_2 = atan2(-0.1e1 / t20 * t19 / 0.2e1, -0.5000000000e0 / t1 * (-0.5e1 * t3 - 0.5e1 * h - 0.1000000000e1 / t20 * t19));

    t1 = tan(eta);
    t2 = t1 * t1;
    t3 = t2 * h;
    t8 = t2 * t2;
    t10 = h * h;
    t16 = t2 * t10;
    t20 = sqrt(-0.25e2 * t10 * t8 * t2 - 0.50e2 * t8 * t10 - 0.25e2 * t16 + 0.4e1 * t2 + 0.4e1 * t8);
    t24 = 0.1e1 / t1;
    double theta2_2 = atan2(0.2500000000e0 / (t2 + 0.1e1) * t24 * (0.5e1 * t3 - t20 + 0.5e1 * h) * (-0.10e2 * t3 - 0.10e2 * h) + 0.5000000000e0 * t24 * (0.25e2 * t16 + 0.25e2 * t10), 0.1250000000e2 * t16 + 0.1250000000e2 * t10 - 0.1e1);

    if (theta1_1 >= 0.0 && theta1_1 <= M_PI && theta2_1 > 50 * (M_PI/180) && theta2_1 <= 160){
        theta[0] = theta1_1;
        theta[1] = theta2_1;
        theta[2] = M_PI - (theta1_1 + theta2_1);
    }
    else{
        theta[0] = theta1_2;
        theta[1] = theta2_2;
        theta[2] = M_PI - (theta1_2 + theta2_2);
    }
    //std::cout << theta[0] * 180 / M_PI << "  " << theta[1] * 180 / M_PI << "  " << theta[2] * 180 / M_PI << std::endl;
    return theta;
}

vector<double> ROBOT::spin(Matrix3d gyro, Matrix3d accel){
    // This Function Should be called in Control loop of choreonoid simple controller
    this->update_sensors(gyro,accel);
    this->send_motion_commands();
}
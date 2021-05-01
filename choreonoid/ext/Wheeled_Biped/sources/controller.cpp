#include "headers/controller.h"

CONTROLLER::CONTROLLER(){
    this->i_eta = 0.0;
    this->i_phi = 0.0;

    this->old_eta = 0.0;
    this->old_phi = 0.0;

    cout << "Controller Object Initialied...\n";
}

double CONTROLLER::phi_control(double phi_d, double phi, double kp, double ki, double kd){
    double error = phi - phi_d;
    this->i_phi += error;
    double deriv = error - this->old_phi;
    this->old_phi = error;
    double speed = kp * error + ki * this->i_phi + kd * deriv;
    return speed;
}

double CONTROLLER::eta_control(double eta_d, double eta, double kp, double ki, double kd){
    double error = eta - eta_d;
    this->i_eta += error;
    double deriv = error - this->old_eta;
    this->old_eta = error;
    double target_eta = kp * error + ki * this->i_eta + kd * deriv;
    return target_eta;
}
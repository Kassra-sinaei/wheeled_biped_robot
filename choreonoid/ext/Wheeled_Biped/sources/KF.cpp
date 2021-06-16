#include "headers/KF.h"

KF::KF(MatrixXd A, MatrixXd B, MatrixXd H, double Q, double R, MatrixXd X0, MatrixXd p_ini){
    this->A = A;
    this->B = B;
    this->H = H;
    this->q = Q;
    this->r = R;
    this->P = p_ini;
    this->x_hat = X0;

    cout << "Kalman Filter initialized...\n";
    
}

MatrixXd KF::getState(){
    return this->x_hat;
}

double* KF::gauss_pdf(){
    // TODO
}

void KF::estimate(MatrixXd U, MatrixXd Y){
    this->predict(U);
    this->update(Y);
}

void KF::predict(MatrixXd u){
    x_bar = A * x_hat + B * u;
    P_bar = A * P * A.transpose() + q * MatrixXd::Identity(P.rows(), P.cols());
}

void KF::update(MatrixXd z){
    MatrixXd K = P_bar * H.transpose() * (r * MatrixXd::Identity(P.rows(),P.cols()) + H * P_bar * H.transpose()).inverse();     // Kalman filter Gain
    x_hat = x_bar + K * (z - H * x_bar);        // state matrice D is zero for our problem
    P = (MatrixXd::Identity(P_bar.rows(), P_bar.cols()) - K * H) * P_bar;
}
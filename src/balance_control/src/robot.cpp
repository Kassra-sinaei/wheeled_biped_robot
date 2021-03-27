#include "robot.h"

robot::robot(double h, double s, double n, double r, double l){
    // Constructor for defining robot's properties
    this->H = h;
    this->S = s;
    this->N = n;
    this->R = r;
    this->L = l;
}

Vector3d robot::rot2rpy(MatrixXd rotation){

    double alpha = atan(rotation(1,0)/rotation(0,0));
    double beta = atan(-rotation(2,0)/(sqrt(rotation(2,1)*rotation(2,1) + rotation(2,2)*rotation(2,2))));
    double gamma = atan(rotation(2,1)/rotation(2,2));
    Vector3d ans;
    ans << alpha, beta, gamma;
    return ans;
}

Vector3d robot::quat2rot(VectorXd quaternion){
    // quaternion = [qx,qy,qz,qw]
    double q0, q1, q2, q3;
    q0 = quaternion(0);
    q1 = quaternion(1);
    q2 = quaternion(2);
    q3 = quaternion(3);

    double r00 = 2 * (q0 * q0 + q1 * q1) - 1;
    double r01 = 2 * (q1 * q2 - q0 * q3);
    double r02 = 2 * (q1 * q3 + q0 * q2);

    double r10 = 2 * (q1 * q2 + q0 * q3);
    double r11 = 2 * (q0 * q0 + q2 * q2) - 1;
    double r12 = 2 * (q2 * q3 - q0 * q1);

    double r20 = 2 * (q1 * q3 - q0 * q2);
    double r21 = 2 * (q2 * q3 + q0 * q1);
    double r22 = 2 * (q0 * q0 + q3 * q3) - 1;

    MatrixXd res(3,3);
    res << r00, r01, r02, r10, r11, r12, r20, r21, r22;
    return res;

}

Vector3d robot::quaternion2euler(VectorXd quaternion){
    // Calculating Euler angles from IMU Quaternion outputs
    return(this->rot2rpy(this->quat2rot(quaternion)));
}

double robot::theta2psi(double theta2){
    // Claculate Knee motor angle
    double a = sqrt(H*H + N*N - 2*H*N*cos(theta2));
    double psi1 = asin(N/H * sin(theta2));
    double psi2 = acos((a*a + R*R - L*L) / (2*L*R));
    return psi1 + psi2;
}

double robot::l_theta2sim(double theta1){
    // Set frame offset for left hip base joint in Gazebo
    return theta1 + (45.0 * M_PI / 180);
}

void robot::set_pose_attitude(Vector3d pose, VectorXd attitude){

    Vector3d euler = this->quaternion2euler(attitude);
    this->base_pose << pose(0),pose(1),pose(2);
    this->base_attitude << euler(0), euler(1), euler(2);
}

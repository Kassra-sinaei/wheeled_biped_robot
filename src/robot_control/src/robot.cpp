#include "robot.h"

robot::robot(double h, double s, double n, double l, double r){ // Test OK
    // Constructor for defining robot's properties
    this->H = h;
    this->S = s;
    this->N = n;
    this->R = r;
    this->L = l;
    //this->vel = 0;
}

MatrixXd robot::quaternion2rotation(Quaterniond quaternion){
    // Calculating Euler angles from IMU Quaternion outputs
    return(quaternion.normalized().toRotationMatrix());
}

Vector3d robot::quaternion2ypr(Quaterniond quaternion){
    // Calculating Euler angles from IMU Quaternion outputs
    return quaternion.normalized().toRotationMatrix().eulerAngles(0,1,2);
}

double robot::theta2psi(double theta2){   //Test OK
    // Claculate Knee motor angle
    double a = sqrt(H*H + N*N - 2*H*N*cos(theta2));
    double psi1 = asin(N/a * sin(theta2));
    double psi2 = acos((a*a + R*R - L*L) / (2*a*R));
    std::cout << psi2 * 180 / M_PI << "  " << psi1 * 180 / M_PI<< std::endl;
    return psi1 + psi2;
}

double robot::l_theta2sim(double theta1){   // Test OK
    // Set frame offset for left hip base joint in Gazebo
    return theta1 + (43.5 * M_PI / 180);
}

void robot::set_pose_attitude(Vector3d pose, Quaterniond attitude){

    MatrixXd euler = this->quaternion2ypr(attitude);
    this->base_pose << pose(0),pose(1),pose(2);
    this->base_attitude << euler(0), euler(1), euler(2);
}

/*
void robot::set_vel(Vector3d vel){
    this->base_vel = vel;
}
*/

#include "robot.h"

Robot::Robot(bool knee_mechanism, double hip, double shank, double n, double r, double diameter, double sample_time){
    this->knee = knee_mechanism;

    this->hip_l = hip;
    this->shank_l = shank;
    this->n_l = n;
    this->r_l = r;
    this->d_wheel = diameter;

    this->dt = sample_time;
    this->time = 0.0;
    this->velocity = Vector3d::Zero(3);
    this->base_R = Matrix3d::Identity(3,3);
    this->base_attitude = Vector3d::Zero(3);
    this->base_pose = Vector3d::Zero(3);
    this->current_eta = -0.1;
    this->old_attitude = this->base_attitude;
}

double Robot::clamp(double value, double high, double low){
    if(value <= high && value >= low)
        return value;
    else if(value < low)
        return low;
    else
        return high;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;
    double M, m, control_rate;
    M = 0.304648;
    m = 5.778;
    control_rate = 0.001;
    Robot WB(true, 0.2, 0.2, 0.08, 0.22, 0.05, 0.001);
    ros::spin();
}
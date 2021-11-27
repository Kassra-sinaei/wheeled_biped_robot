#pragma once

#include "lqr.h"
#include "robot_control/Joint_cmd.h"

using namespace Eigen;
using namespace std;

class Robot{
    public:
        Robot(bool knee_mechanism, double hip, double shank, double n, double r, double diameter, double dt);
        bool spin(Vector3d position, Vector3d velocity, Matrix3d rotation, MatrixXd x_d, double height);
    private:
        //Joint Lengths
        double hip_l;
        double shank_l;
        double r_l;
        double l_l;
        double n_l;
        double d_wheel;

        Vector3d dv;
        Matrix3d base_R;
        Vector3d base_attitude;
        Vector3d ang_vel;
        Vector3d velocity;
        Vector3d base_pose;
        Vector3d old_attitude;
        double current_eta;

        double dt;
        double time;

        double* robot_pose( double h);
        double knee_linkage(double theta);
        void update_sensors(Vector3d gyro, Vector3d accelerometer, double omega_r, double omega_l, vector<double> config);
        vector<double> send_motion_commands(MatrixXd state, double height);
        void forward_kinematic(vector<double> config);
        double clamp(double value, double high, double low);
        double zero_eta(double height);

        bool knee;
        fstream file;

        ros::NodeHandle nh;
        ros::ServiceServer joint_cmnd;
};
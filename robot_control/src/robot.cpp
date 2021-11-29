#include "../include/robot_control/robot.h"

Robot::Robot(bool knee_mechanism, double hip, double shank, double n, double l, double r, double diameter, double sample_time){
    this->knee = knee_mechanism;

    this->hip_l = hip;
    this->shank_l = shank;
    this->n_l = n;
    this->r_l = r;
    this->l_l = l;
    this->d_wheel = diameter;

    this->current_height = 0.0;
    this->current_gain1 = MatrixXd::Zero(1, 4);
    this->current_gain2 = MatrixXd::Zero(1, 2);
    this->decouple << 0.5, 0.5, 0.5, -0.5;

    joint_cmd = nh.advertiseService("/joint_cmd", &Robot::spinOnline, this);
    lqr_gain = nh.serviceClient<robot_control::Gain>("/lqr_gain");

    ROS_INFO("Dynamic and Control Node is Online...");
}

bool Robot::spinOnline(robot_control::Joint_cmd::Request &req, robot_control::Joint_cmd::Response &res){
    double eta = this->zero_eta(req.height);
    double* config = this->robot_pose(eta,req.height);
    double q0, q1;
    q0 = config[2] - 43.0 * M_PI / 180.0;       // Hip Pitch
    if (!this->knee)
        q1 = -79.9 * M_PI / 180.0 + config[1];      // Simple Knee Mechanism
    else
        q1 = 127.0 * M_PI / 180.0 - knee_linkage(config[1]);    // Link L joint position
    
    VectorXd state1(4);
    Vector2d state2;
    state1 << req.x, req.x_d, req.theta, req.theta_d;
    state2 << req.delta, req.delta_d;
    // Update LQR gain if there is a change in control model
    if(abs(req.height - this->current_height) > 0.01){
        this->current_height = req.height;
        robot_control::Gain gain_msg;
        gain_msg.request.l = this->pend_length(this->current_height);
        this->lqr_gain.call(gain_msg);

        this->current_gain1 << gain_msg.response.K[0], gain_msg.response.K[1], gain_msg.response.K[2], gain_msg.response.K[3];
        this->current_gain2 <<  gain_msg.response.K[4], gain_msg.response.K[5];

        ROS_INFO("LQR Gains Updated...");
    }
    
    VectorXd desired1(4);
    desired1 << req.pos, 0.0, 0.0, 0.0;
    Vector2d desired2(req.yaw,0.0);
    Vector2d effort = this->decouple * 
                    Vector2d((this->current_gain1 * (desired1-state1))(0),
                            (this->current_gain2 * (desired2-state2))(0));
    res.config[0] = -q0;
    res.config[1] = -q1;
    res.config[2] = -effort(1);
    res.config[3] = q0;
    res.config[4] = q1;
    res.config[5] = effort(0);

    //ROS_INFO("Joint Angles Returned.");
    return true;
}

double Robot::zero_eta(double height){
    // This Values should be used for the Inverse Kineamtic
    if(height > 0.195 && height < 0.205)
        return -0.139;
    else if(height > 0.205 && height < 0.215)
        return -0.131;
    else if(height > 0.215 && height < 0.225)
        return -0.123;
    else if(height > 0.225 && height < 0.235)
        return -0.116;
    else if(height > -0.235 && height < 0.245)
        return -0.110;
    else if(height > 0.245 && height < 0.255)
        return -0.103;
    else if(height > 0.255 && height < 0.265)
        return -0.097;
    else if(height > 0.265 && height < 0.275)
        return -0.091;
    else if(height > 0.275 && height < 0.285)
        return -0.086;
    else if(height > 0.285 && height < 0.295)
        return -0.081;
    else if(height > 0.295 && height < 0.305)
        return -0.076;
    else if(height > 0.305 && height < 0.315)
        return -0.071;
    
    return -0.1;
}

double Robot::pend_length(double height){
    // Final Results of Pendulum's length in model
    if(height > 0.195 && height < 0.205)
        return 0.1712;
    else if(height > 0.205 && height < 0.215)
        return 0.1794;
    else if(height > 0.215 && height < 0.225)
        return 0.1877;
    else if(height > 0.225 && height < 0.235)
        return 0.1961;
    else if(height > 0.235 && height < 0.245)
        return 0.2044;
    else if(height > 0.245 && height < 0.255)
        return 0.2127;
    else if(height > 0.255 && height < 0.265)
        return 0.2210;
    else if(height > 0.265 && height < 0.275)
        return 0.2294;
    else if(height > 0.275 && height < 0.285)
        return 0.2377;
    else if(height > 0.285 && height < 0.295)
        return 0.2461;
    else if(height > 0.295 && height < 0.305)
        return 0.2545;
    else if(height > 0.305 && height < 0.315)
        return 0.2630;

    return 0.21;
}

double Robot::knee_linkage(double theta){
    // Analytic Solve of Knee 4-bar Mechanism
    if (theta == 0.0){theta = 0.0001;}       // to avoid devision by zero
    double a = sqrt(hip_l*hip_l + n_l*n_l - 2*hip_l*n_l*cos(theta));
    double psi1 = asin(n_l/a * sin(theta));
    double psi2 = acos((a*a + r_l*r_l - l_l*l_l) / (2*a*r_l));
    return psi1 + psi2;
}

double* Robot::robot_pose(double eta, double h){
    if (eta == 0.0) {eta = 0.0001;}

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
    //cout << theta[0] * 180 / M_PI << "  " << theta[1] * 180 / M_PI << "  " << theta[2] * 180 / M_PI << std::endl;
    return theta;
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
    ros::init(argc, argv, "dynamic_control");
    ros::NodeHandle nh;
    double M, m, control_rate;
    M = 0.304648;
    m = 5.778;
    control_rate = 0.001;
    Robot WB(true, 0.2, 0.2, 0.08, 0.22, 0.05, 0.1, control_rate);
    ros::spin();
}
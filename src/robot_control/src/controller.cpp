#include "controller.h"

controller::controller(robot *robot_pointer){       

            rbt = robot_pointer;

            ros::NodeHandle nh;
            ros::Rate loop_rate(50);

            row_imu = nh.subscribe("imu",5,&controller::imu_rpy,this);
            odom = nh.subscribe("base_vel",5,&controller::set_vel,this);
            joint1_pub = nh.advertise<std_msgs::Float64>("/joint1_position_controller/command", 50);
            joint2_pub = nh.advertise<std_msgs::Float64>("/joint1_position_controller/command", 50);
            joint3_pub = nh.advertise<std_msgs::Float64>("/joint1_position_controller/command", 50);
            joint4_pub = nh.advertise<std_msgs::Float64>("/joint1_position_controller/command", 50);
            joint5_pub = nh.advertise<std_msgs::Float64>("/joint1_position_controller/command", 50);
            joint6_pub = nh.advertise<std_msgs::Float64>("/joint1_position_controller/command", 50);

            imu_y = nh.advertise<std_msgs::Float64>("test_imu", 50);
}

void controller::imu_rpy(const sensor_msgs::Imu::ConstPtr& msg){
            Quaterniond quaternion(msg->orientation.w,
                                    msg->orientation.x,
                                    msg->orientation.y,
                                    msg->orientation.z);
            Vector3d pose(0.0,0.0,0.0);
            rbt->set_pose_attitude(pose,quaternion);
} 

void controller::set_vel(const nav_msgs::Odometry &msg){
            Vector3d velocity;
            velocity << msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z;
            rbt->base_vel = velocity;
            //ROS_INFO("I have been executed too");
}

double controller::vel_controller(double target_vel, double current_vel, double kp, double ki, double kd){
    double error = current_vel - target_vel;
    this->i_vel += error;
    double deriv = error - this->previous_error_vel;
    this->previous_error_vel = error;
    double etha = kp * error + ki * this->i_vel + kd * deriv;
    return etha;
}

float* controller::leg_config(double height, double etha){
    float theta[3];


    return theta;
}

double controller::wheel_speed(double current_phi, double desired_phi, double kp, double ki, double kd){
    double error = current_phi - desired_phi;
    this->i_phi += error;
    double deriv = error - this->previous_error_phi;
    this->previous_error_phi = error;
    double speed = kp * error + ki * this->i_vel + kd * deriv;
    return speed;
}
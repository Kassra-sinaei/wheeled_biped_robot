#include "robot.h"

#include "ros/ros.h"
#include "ros/console.h"
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#include <iostream>
#include "cmath"
#include <math.h>
#include <vector>

#include"Eigen/Dense"
#include "Eigen/eiquadprog.h"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;


class balanceController
{
    public:
        balanceController(robot *robot_pointer){       

            rbt = robot_pointer;

            ros::NodeHandle nh;
            ros::Rate loop_rate(50);

            row_imu = nh.subscribe("imu",5,&balanceController::imu_rpy,this);
            joint1_pub = nh.advertise<std_msgs::Float64>("/joint1_position_controller/command", 50);
            joint2_pub = nh.advertise<std_msgs::Float64>("/joint1_position_controller/command", 50);
            joint3_pub = nh.advertise<std_msgs::Float64>("/joint1_position_controller/command", 50);
            joint4_pub = nh.advertise<std_msgs::Float64>("/joint1_position_controller/command", 50);
            joint5_pub = nh.advertise<std_msgs::Float64>("/joint1_position_controller/command", 50);
            joint6_pub = nh.advertise<std_msgs::Float64>("/joint1_position_controller/command", 50);

            imu_y = nh.advertise<std_msgs::Float64>("test_imu", 50);
        }

    private:
        ros::Subscriber row_imu;

        ros::Publisher joint1_pub;
        ros::Publisher joint2_pub;
        ros::Publisher joint3_pub;
        ros::Publisher joint4_pub;
        ros::Publisher joint5_pub;
        ros::Publisher joint6_pub;

        ros::Publisher imu_y;

        robot *rbt;

        void imu_rpy(const sensor_msgs::Imu::ConstPtr& msg){
            Quaterniond quaternion(msg->orientation.w,
                                    msg->orientation.x,
                                    msg->orientation.y,
                                    msg->orientation.z);
            Vector3d pose(0.0,0.0,0.0);
            rbt->set_pose_attitude(pose,quaternion);
            //cout << rbt->get_attitude()(1) << endl <<"--------" <<endl;

        }   
};


int main(int argc, char **argv)
{
    ros::init(argc,argv,"wb_controller");

    robot wb(0.2,0.2,0.08,0.22,0.05);
    robot *wb_ptr;
    wb_ptr = &wb; 
    //while(ros::ok())
    balanceController BC(wb_ptr);
    
    ros::spin();

    return 0;
    
}
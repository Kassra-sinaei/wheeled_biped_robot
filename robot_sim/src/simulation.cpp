#include <cnoid/SimpleController>
#include <cnoid/Sensor>
#include <cnoid/BodyLoader>
#include <eigen3/Eigen/Eigen>

#include "robot_control/Joint_cmd.h"
#include "robot_control/jump.h"
#include <ros/ros.h>

#include "fstream"

using namespace std;
using namespace cnoid;
using namespace Eigen;

const double pgain[] = {50.0, 50.0, 0.0, 50.0, 50.0, 0.0};
const double dgain[] = {1.0,1.0,0.0,1.0,1.0,0.0};
const double igain[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

double max_torque = 1.50;

class WBController: public SimpleController{

public:
    BodyPtr ioBody;
    double dt;
    string opt;
    ros::NodeHandle nh;
    ros::ServiceClient simSpin;

    virtual bool initialize(SimpleControllerIO* io) override{
        
        ioBody = io->body();
        dt = io->timeStep();
        iteration = 0;
        double sim_duration = 10.0;
        input_height = new float[int(sim_duration/dt)];

        simSpin = nh.serviceClient<robot_control::Joint_cmd>("/joint_cmd");

        // Getting input values
        opt = io->optionString();
        if(opt.length() > 0){
            vector<float> inputs;
            readOptions(opt, inputs);
            input_yaw = inputs[2];
            input_pos = inputs[1];
            if(inputs[0] == -1)
                for(int i =0; i < sim_duration/dt; i ++)
                    input_height[i] = 0.25 + 0.07 * cos(2 * M_PI/sim_duration * i * dt);
            else if(inputs[0] == -2){
                ofstream height_file;
                height_file.open("/home/kassra/Thesis/choreonoid_ws/src/thesis/log/h_robot.csv");
                double* height_traj;
                Jump j_gen(dt);
                int len;
                height_traj = j_gen.generateTraj(inputs[1], 0.0, 0.0, inputs[6], inputs[2], inputs[3], inputs[4], inputs[5], len);
                for(int i =0; i < sim_duration/dt; i ++)
                    input_height[i] = inputs[1];
                for(int i = inputs[2] / dt; (i < (len + inputs[2] / dt)) && (i < sim_duration / dt); i ++){
                    input_height[i] = height_traj[i - int(inputs[2] / dt)];
                    height_file << i * dt << ", " << input_height[i] << endl;    
                }
                input_yaw = 0.0;
                input_pos = 0.0;
                height_file.close();
                delete height_traj;
            }
            else
                for(int i =0; i < sim_duration/dt; i ++)
                    input_height[i] = inputs[0];
            //input_yaw = inputs[2];
            //input_pos = inputs[1];
        }
        else{
            // default values
            for(int i =0; i < sim_duration/dt; i ++)
                input_height[i] = 0.23;
            input_pos = 0.0;
            input_yaw = 0.0;
            ROS_WARN("No Input Command, Using Default Values");
        }
        cout << "Inputs: " << input_height[0] << ", " << input_pos << ", " << input_yaw << endl;
        accelSensor = ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");
        io->enableInput(accelSensor);
        gyro = ioBody->findDevice<RateGyroSensor>("WaistGyro");
        io->enableInput(gyro);

        // Enabling Joint Torque Control IO
        for(int i=0; i < 6; ++i){
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
        }

        // Enabling Base Link 
        io->enableInput(io->body()->rootLink(), LINK_POSITION);
        previous_position = ioBody->rootLink()->position().translation();

        position.open("/home/kassra/Thesis/choreonoid_ws/src/thesis/log/robot_pos.csv",ios::out);
        orientation.open("/home/kassra/Thesis/choreonoid_ws/src/thesis/log/robot_tilt.csv",ios::out);

        return true;
    }

    virtual bool control() override{
        // dv = accelSensor->dv();
        this->tilt += gyro->w()(1) * this->dt;
        Vector3d p = ioBody->rootLink()->position().translation();
        Matrix3d R = ioBody->rootLink()->position().rotation();

        Vector3d current_rot = R.eulerAngles(0, 1, 2);
        Vector3d angular_vel = (current_rot - previous_rot)/dt;
        Vector3d vel = (p-previous_position)/dt;
        position << p(0) << "," << p(1) << "," << p(2) << "\n";
        orientation << tilt << "," << current_rot(0) << "," << current_rot(1) << "," << current_rot(2) << endl;
        
        robot_control::Joint_cmd state;
        state.request.x = sqrt(pow(p(0),2) + pow(p(1),2));
        state.request.x_d = sqrt(pow(vel(0),2) + pow(vel(1),2));
        state.request.theta = this->tilt;
        state.request.theta_d = gyro->w()(1);
        //state.request.delta = current_rot(2);
        state.request.delta = 0;
        //state.request.delta_d = angular_vel(2);
        state.request.delta_d = 0;
        state.request.height = input_height[iteration];
        state.request.pos = input_pos;
        state.request.yaw = input_yaw;
        simSpin.call(state);
        for(int i = 0; i < 6; i ++)
            qref[i] = state.response.config[i];

        if(iteration % 500 == 0)
            cout << qref[0] << "," << qref[1] << "," << qref[2] << ", " << qref[5] << endl;
        previous_position = p;
        previous_rot = current_rot;

        // Send motor torques
        for(int i=0; i < 6; ++i){
            Link* joint = ioBody->joint(i);
            double q = joint->q();
            if (i == 2 || i == 5){
                if(abs(qref[i]) <= max_torque)
                    joint->u() = qref[i];
                else if(qref[i] > 0)
                    joint->u() = max_torque;
                else
                    joint->u() = -max_torque;
                qold[i] = q;
                //file_real << qref[i] << ",";
                continue;
            }
            //file_real << endl;
            double dq = (q - qold[i]) / dt;
            qi[i] += qref[i] - q;
            double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i] + qi[i] * igain[i];
            qold[i] = q;
            joint->u() = u;
        }

        iteration ++;
        return true;
    }

    void readOptions(string opt, vector<float> (&out)){
        stringstream ss(opt);
        float i;

        while (ss >> i){
            out.push_back(i);

            if (ss.peek() == ',')
            ss.ignore();
        }
    }

private:
    int iteration;
    float* input_height;
    float input_yaw;
    float input_pos;

    AccelerationSensor* accelSensor;
    RateGyroSensor* gyro;

    //ROBOT* WB;
    Vector3d previous_position;

    Vector3 dv;
    Vector3 previous_rot;
    double tilt;

    vector<double> qref{0.0,0.0,0.0,0.0,0.0,0.0};
    vector<double> qold{0.0,0.0,0.0,0.0,0.0,0.0};
    vector<double> qi{0.0,0.0,0.0,0.0,0.0,0.0};

    ofstream orientation;
    ofstream position;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(WBController)
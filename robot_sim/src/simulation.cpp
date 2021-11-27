#include <cnoid/SimpleController>
#include <cnoid/Sensor>
#include <cnoid/BodyLoader>
#include <eigen3/Eigen/Eigen>

#include "robot_control/Joint_cmd.h"
#include <ros/ros.h>

#include "fstream"

using namespace std;
using namespace cnoid;
using namespace Eigen;

const double pgain[] = {50.0, 50.0, 0.0, 50.0, 50.0, 0.0};
const double dgain[] = {1.0,1.0,0.0,1.0,1.0,0.0};
const double igain[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

double max_torque = 1.5;

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

        simSpin = nh.serviceClient<robot_control::Joint_cmd>("/join_cmd");

        // Getting input values
        opt = io->optionString();
        if(opt.length() > 0){
            vector<float> inputs;
            readOptions(opt, inputs);
            input_height = inputs[0];
            input_velocity = inputs[1];
        }
        else{
            // default values
            input_height = 0.27;
            input_velocity = 0.0;
        }
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

        //file_real.open("log/robot state.csv",ios::out);

        return true;
    }

    virtual bool control() override{
        // dv = accelSensor->dv();
        double tilt = gyro->w()(1);
        Vector3d p = ioBody->rootLink()->position().translation();
        Matrix3d R = ioBody->rootLink()->position().rotation();

        Vector3d current_rot = R.eulerAngles(0, 1, 2);
        Vector3d angular_vel = (current_rot - previous_rot)/dt;
        Vector3d vel = (p-previous_position)/dt;
        //file_real << vel(0) << "," << vel(1) << "," << vel(2) << "\n";
        //file_real << vel(0) << "," << WB->base_attitude(1) << endl;
        
        robot_control::Joint_cmd state;
        state.request.x = sqrt(pow(p(0),2) + pow(p(1),2));
        state.request.x_d = sqrt(pow(vel(0),2) + pow(vel(1),2));
        state.request.theta = tilt;
        state.request.theta_d = (tilt - previous_tilt)/dt;
        state.request.delta = current_rot(2);
        state.request.delta_d = angular_vel(2);
        simSpin.call(state);

        previous_position = p;
        previous_rot = current_rot;
        previous_tilt = tilt;

        // Send motor torques
        for(int i=0; i < 6; ++i){
            Link* joint = ioBody->joint(i);
            double q = joint->q();
            if (i == 2 || i == 5){
                joint->u() = state.response.config[i];
                qold[i] = q;
                //file_real << qref[i] << ",";
                continue;
            }
            //file_real << endl;
            double dq = (q - qold[i]) / dt;
            qi[i] += state.response.config[i] - q;
            double u = (state.response.config[i] - q) * pgain[i] + (0.0 - dq) * dgain[i] + qi[i] * igain[i];
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
    double input_height;
    double input_velocity;

    AccelerationSensor* accelSensor;
    RateGyroSensor* gyro;

    //ROBOT* WB;
    Vector3d previous_position;

    Vector3 dv;
    Vector3 previous_rot;
    double previous_tilt;

    vector<double> qref{0.0,0.0,0.0,0.0,0.0,0.0};
    vector<double> qold{0.0,0.0,0.0,0.0,0.0,0.0};
    vector<double> qi{0.0,0.0,0.0,0.0,0.0,0.0};

    ofstream file_real;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(WBController)
#include <cnoid/SimpleController>
#include <cnoid/Sensor>
#include <cnoid/BodyLoader>
#include <eigen3/Eigen/Eigen>
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

    virtual bool initialize(SimpleControllerIO* io) override{
        
        ioBody = io->body();
        dt = io->timeStep();
        iteration = 0;

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
        dv = accelSensor->dv();
        angular_velocity = gyro->w();
        Vector3d p = ioBody->rootLink()->position().translation();
        Matrix3d R = ioBody->rootLink()->position().rotation();

        Vector3d vel = (p-previous_position)/dt;
        //file_real << vel(0) << "," << vel(1) << "," << vel(2) << "\n";
        //file_real << vel(0) << "," << WB->base_attitude(1) << endl;
        previous_position = p;

        // Send motor torques
        for(int i=0; i < 6; ++i){
            Link* joint = ioBody->joint(i);
            double q = joint->q();
            if (i == 2 || i == 5){
                joint->u() = qref[i];
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
    double input_height;
    double input_velocity;

    AccelerationSensor* accelSensor;
    RateGyroSensor* gyro;

    //ROBOT* WB;
    Vector3d previous_position;

    Vector3 dv;
    Vector3 angular_velocity;

    vector<double> qref{0.0,0.0,0.0,0.0,0.0,0.0};
    vector<double> qold{0.0,0.0,0.0,0.0,0.0,0.0};
    vector<double> qi{0.0,0.0,0.0,0.0,0.0,0.0};

    ofstream file_real;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(WBController)
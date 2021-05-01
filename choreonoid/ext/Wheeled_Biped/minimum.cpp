#include <cnoid/SimpleController>
#include <cnoid/Sensor>
#include <cnoid/BodyLoader>

#include"Eigen/Dense"
#include "Eigen/eiquadprog.h"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include <Eigen/Geometry>

#include "iostream"
#include "fstream"
#include <vector>


using namespace std;
using namespace cnoid;
using namespace Eigen;

const double pgain[] = {35.0, 22.0, 0.0, 35.0, 22.0, 0.0};

const double dgain[] = {1.7,2.0,0.0,1.7,2.0,0.0};


class WB1MinimumController : public SimpleController
{
    BodyPtr ioBody;
    double dt;

    AccelerationSensor* accelSensor;
    RateGyroSensor* gyro;
    Link* gyro_link;

    std::vector<double> qref;
    std::vector<double> qold;

    double i_phi;
    double previous_phi_error;

public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        myfile.open ("error.txt");
        if (myfile.is_open()) 
            cout << "file is open" << endl;

        ioBody = io->body();
        dt = io->timeStep();

        accelSensor = ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");
        io->enableInput(accelSensor);
        gyro = ioBody->findDevice<RateGyroSensor>("WaistGyro");
        io->enableInput(gyro);
        gyro_link = gyro->link();

        for(int i=0; i < 6; ++i){
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
            qref.push_back(joint->q());
        }
        qold = qref;

        i_phi = 0;
        previous_phi_error = 0;

        return true;
    }

    virtual bool control() override
    {
        dv = accelSensor->dv();
        attitude = gyro->w();
        
        qref[2] = -attitude_controller(attitude(1), 0.0);
        qref[5] = attitude_controller(attitude(1), 0.0);

        for(int i=0; i < 6; ++i){
            Link* joint = ioBody->joint(i);
            if (i == 2 || i == 5){
                joint->u() = qref[i];
                continue;
            }
            double q = joint->q();
            double dq = (q - qold[i]) / dt;
            double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
            qold[i] = q;
            joint->u() = u;
            //joint->q() = qref[i];
        }

        return true;
    }

private:
    VectorXd dv;
    MatrixXd attitude;
    ofstream myfile;

    double attitude_controller(double phi, double desired_phi, double kp = 6.7, double ki = 0.005 , double kd = 0.06){
        double error = phi - desired_phi;
        myfile << error << "\n";
        this->i_phi += error;
        double deriv = error - this->previous_phi_error;
        this->previous_phi_error = error;
        return kp * error + ki * this->i_phi + kd * deriv;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(WB1MinimumController)
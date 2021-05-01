#include <cnoid/SimpleController>
#include <cnoid/Sensor>
#include <cnoid/BodyLoader>

#include "headers/robot.h"

using namespace std;
using namespace cnoid;

const double pgain[] = {35.0, 22.0, 0.0, 35.0, 22.0, 0.0};

const double dgain[] = {1.7,2.0,0.0,1.7,2.0,0.0};

class WBController: public SimpleController{

public:
    BodyPtr ioBody;
    double dt;

    AccelerationSensor* accelSensor;
    RateGyroSensor* gyro;

    ROBOT* WB;

    Vector3d dv;
    Vector3d attitude;

    virtual bool initialize(SimpleControllerIO* io) override{

        WB = new ROBOT(0.2,0.2,0.08,0.22,0.05);

        accelSensor = ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");
        io->enableInput(accelSensor);
        gyro = ioBody->findDevice<RateGyroSensor>("WaistGyro");
        io->enableInput(gyro);

        for(int i=0; i < 6; ++i){
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JOINT_TORQUE);
        }

        return true;
    }

    virtual bool control() override{
        dv = accelSensor->dv();
        attitude = gyro->w();
        WB->spin(dv, attitude);
        return true;
    }
private:

};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(WBController)
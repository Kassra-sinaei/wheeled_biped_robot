#include <cnoid/SimpleController>
#include <cnoid/Sensor>
#include <cnoid/BodyLoader>

#include "iostream"
#include "fstream"
#include <vector>

using namespace std;
using namespace cnoid;

const double pgain[] = {35.0, 22.0, 0.0, 35.0, 22.0, 0.0};

const double dgain[] = {1.7,2.0,0.0,1.7,2.0,0.0};

class WBController: public SimpleController{

public:
    BodyPtr ioBody;
    double dt;

    virtual bool initialize(SimpleControllerIO* io) override{
        return true;
    }

    virtual bool control() override{
        return true;
    }
private:

};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(WBController)
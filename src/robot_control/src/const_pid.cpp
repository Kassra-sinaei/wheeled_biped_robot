#include "controller.h"

using namespace Eigen;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"wb_controller");

    robot wb(0.2,0.2,0.08,0.22,0.05);
    robot *wb_ptr;
    wb_ptr = &wb; 
    //while(ros::ok())
    controller BC(wb_ptr);
    //cout << "I have been executed" << endl;
    ros::spin();

    return 0;
    
}
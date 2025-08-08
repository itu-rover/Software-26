/*
    author: Yunus Emre Akar
*/
#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <arm23_control/arm23_hw_interface.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "arm23_hw_interface");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    std::shared_ptr<arm23::arm23HWInterface> arm23_hw_interface(new arm23::arm23HWInterface(nh));
    arm23_hw_interface->init();

    ros_control_boilerplate::GenericHWControlLoop control_loop(nh, arm23_hw_interface);
    control_loop.run();

    return 0;
}
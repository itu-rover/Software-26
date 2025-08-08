#ifndef ARM23_HW_INTERFACE_H
#define ARM23_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>


namespace arm23
{
    class arm23HWInterface : public ros_control_boilerplate::GenericHWInterface
    {
    public:
        arm23HWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

        virtual void init();
        virtual void read(ros::Duration& elapsed_time);
        virtual void write(ros::Duration& elapsed_time);
        virtual void enforceLimits(ros::Duration& period);

    protected:
        void feedback(const std_msgs::Float32MultiArray::ConstPtr &msg);
        double axis[6];
        double zero_pos[6];
        bool reset_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        ros::ServiceServer reset_service_;

    };
}
#endif
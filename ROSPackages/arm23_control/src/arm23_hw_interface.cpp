#include <arm23_control/arm23_hw_interface.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>

ros::Publisher pub;
ros::Subscriber sub;

namespace arm23
{
    arm23HWInterface::arm23HWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
            : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
    {
        sub = nh.subscribe<std_msgs::Float32MultiArray>("joints_feedback", 1000, &arm23HWInterface::feedback, this);
        pub = nh.advertise<std_msgs::Float32MultiArray>("joint_commands", 1000);
        reset_service_ = nh.advertiseService("/hardware_interface/reset_pos", &arm23HWInterface::reset_srv, this);
        zero_pos[0] = 0;
        zero_pos[1] = 0;
        zero_pos[2] = 0;
        zero_pos[3] = 0;
        zero_pos[4] = 0;
        zero_pos[5] = 0;
    }

    void arm23HWInterface::init()
    {
        ros_control_boilerplate::GenericHWInterface::init();
        ROS_INFO("Naim interface initialized");
    }
    void arm23HWInterface::feedback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        axis[0] = msg->data[0];
        axis[1] = msg->data[1];
        axis[2] = msg->data[2];
        axis[3] = msg->data[3];
        axis[4] = msg->data[4];
        axis[5] = msg->data[5];
    }
    bool arm23::arm23HWInterface::reset_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
        zero_pos[0] = axis[0];
        zero_pos[1] = axis[1]+0.436;
        //zero_pos[1] = axis[1];
        zero_pos[2] = axis[2];
        zero_pos[3] = axis[3];
        zero_pos[4] = axis[4];
        zero_pos[5] = axis[5]+1.57;
        res.success = true;
        res.message = "Position reset";
        return true;
    }

    void arm23HWInterface::read(ros::Duration & elapsed_time)
    {
        joint_position_[0] = axis[0]-zero_pos[0];
        joint_position_[1] = axis[1]-zero_pos[1];
        joint_position_[2] = axis[2]-zero_pos[2];
        joint_position_[3] = axis[3]-zero_pos[3];
        joint_position_[4] = axis[4]-zero_pos[4];
        joint_position_[5] = axis[5]-zero_pos[5];
        
    }

    void arm23HWInterface::write(ros::Duration & elapsed_time)
    {
        // Send joint commands
        std_msgs::Float32MultiArray pos_command;

        for (int i = 0; i < 6; i++)
        {
            pos_command.data.push_back((joint_position_command_[i])+zero_pos[i]);
        }

        pub.publish(pos_command);
    }
    void arm23HWInterface::enforceLimits(ros::Duration &period)
    {
        // joint_pos_command_[0] = (joint_pos_command_[0] > 3.14 ) ? 3.14 : joint_pos_command_[0];
        // // Implement joint limits
    }
};

/**
 * @author B. Burak Payzun
 * @date 2022-03-10
 *
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <chrono>
#include <cmath>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Trigger.h>

const std::string PLANNING_GROUP = "manipulator";

double threshold = 0.8;
class Interface{
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        ros::Subscriber sub_;
        ros::ServiceServer service_;

        std::string ee_link_name_;

        moveit::planning_interface::MoveGroupInterfacePtr move_group_;

        robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
        planning_scene::PlanningScenePtr planning_scene_;
        
        // Raw pointer because we are not responsible with its life time.
        const robot_state::JointModelGroup* joint_model_group_; 

        kinematics::KinematicsBaseConstPtr solver_instance_;

        std::chrono::_V2::system_clock::time_point t1;
        std::vector<double> current_joint_positions;
        std::vector<double> joint_group_positions;

        kinematics::KinematicsResult ik_res;
        kinematics::KinematicsQueryOptions options;
        std::vector<geometry_msgs::Pose> poses;
        moveit::core::RobotStatePtr current_state;

        geometry_msgs::PoseStamped current_pose;
        bool fetched_current_pose = false;
        geometry_msgs::Pose target_pose;

        bool reset_target_position(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
        {
            current_pose = move_group_->getCurrentPose(ee_link_name_);
            current_state->copyJointGroupPositions(joint_model_group_, joint_group_positions);
            current_joint_positions = move_group_->getCurrentJointValues(); //15ms
            
            
            trajectory_msgs::JointTrajectory traj;
            traj.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

            trajectory_msgs::JointTrajectoryPoint pts;
            pts.positions = current_joint_positions;
            pts.time_from_start = ros::Duration(0.005);

            traj.points.push_back(pts);

            pub_.publish(traj);

            res.success = true;
            res.message = "reset success";
            return true;
        }

        void callback(const geometry_msgs::TwistStampedConstPtr &data){
            if (!fetched_current_pose) return;

            using namespace std::chrono;
            current_pose.pose.position.x = current_pose.pose.position.x + data->twist.linear.x;
            current_pose.pose.position.y = current_pose.pose.position.y + data->twist.linear.y;
            current_pose.pose.position.z = current_pose.pose.position.z + data->twist.linear.z;

            tf2::Quaternion current_q;
            tf2::convert(current_pose.pose.orientation, current_q);

            tf2::Quaternion rot_q;
            rot_q.setRPY(data->twist.angular.x, data->twist.angular.y, data->twist.angular.z);

            static tf2_ros::TransformBroadcaster br;
            geometry_msgs::TransformStamped transformStamped;

            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = "target";
            transformStamped.transform.translation.x = current_pose.pose.position.x;
            transformStamped.transform.translation.y = current_pose.pose.position.y;
            transformStamped.transform.translation.z = current_pose.pose.position.z;
            tf2::Quaternion q;
            q = rot_q*current_q;
            q.normalize();


            tf2::convert(q, current_pose.pose.orientation);

            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();
            br.sendTransform(transformStamped);

            current_joint_positions = move_group_->getCurrentJointValues(); //15ms

            poses.push_back(current_pose.pose);
            
            bool ik_has_solution = current_state->setFromIK(joint_model_group_, current_pose.pose);
            if(!ik_has_solution){
                ROS_INFO_NAMED("tutorial", "IK SOLUTION NOT FOUND");
                return;
            }
            // if(planning_scene_->isStateColliding(*current_state, PLANNING_GROUP)){
            //     ROS_INFO_NAMED("tutorial", "SOLUTION IN COLLISION");
            //     return;
            // }
            
            current_state->copyJointGroupPositions(joint_model_group_, joint_group_positions);


            ROS_INFO_NAMED("tutorial", "Current position:  %lf %lf %lf %lf %lf %lf",   current_joint_positions[0],
                                                                                                            current_joint_positions[1],
                                                                                                            current_joint_positions[2],
                                                                                                            current_joint_positions[3],
                                                                                                            current_joint_positions[4],
                                                                                                            current_joint_positions[5]);
            ROS_INFO_NAMED("tutorial", "Solution found: %lf %lf %lf %lf %lf %lf",   joint_group_positions[0],
                                                                                                            joint_group_positions[1],
                                                                                                            joint_group_positions[2],
                                                                                                            joint_group_positions[3],
                                                                                                            joint_group_positions[4],
                                                                                                            joint_group_positions[5]);

            for(int i=0; i < joint_group_positions.size(); i++){
                if(std::fabs(std::fmod(joint_group_positions[i] - current_joint_positions[i], 3.14)) > threshold){
                    std::cout<<(joint_group_positions[i] - current_joint_positions[i])<<std::endl;
                    return;
                }
            }

            trajectory_msgs::JointTrajectory traj;

            traj.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
            trajectory_msgs::JointTrajectoryPoint pts;
            pts.positions = joint_group_positions;
            pts.time_from_start = ros::Duration(0.005);

            traj.points.push_back(pts);
            pub_.publish(traj);
        }

    public:
    Interface(){
        ros::AsyncSpinner spinner(4);
        spinner.start();

        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
            
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_model_loader_);
        planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
        planning_scene_ = planning_scene_monitor_->getPlanningScene();

        joint_model_group_ = move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        
        solver_instance_ = joint_model_group_->getSolverInstance();
        
        ee_link_name_ = move_group_->getEndEffectorLink();

        current_pose = move_group_->getCurrentPose(ee_link_name_); //15ms
        fetched_current_pose = true;

        current_state = move_group_->getCurrentState();
        ROS_INFO_NAMED("tutorial", "Movegroup interface is successfully initialized!");
        pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/manipulator_controller/command/", 10);

        sub_ = nh_.subscribe("/servo_server/delta_twist_cmds", 10, &Interface::callback, this);
        service_ = nh_.advertiseService("/servo_server/reset_target", &Interface::reset_target_position, this);

        ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_->getPlanningFrame().c_str());
        ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_->getEndEffectorLink().c_str());
        ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
        std::copy(move_group_->getJointModelGroupNames().begin(), move_group_->getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));

        ros::waitForShutdown();
    }
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_server");

    Interface();
    return 0;
}

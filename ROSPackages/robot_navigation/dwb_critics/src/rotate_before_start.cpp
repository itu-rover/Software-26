#include <dwb_critics/rotate_before_start.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <dwb_critics/rotate_to_goal.h>
#include <nav_2d_utils/parameters.h>


PLUGINLIB_EXPORT_CLASS(dwb_critics::RotateBeforeStartCritic, dwb_local_planner::TrajectoryCritic)

inline double hypot_sq(double dx, double dy)
{
  return dx * dx + dy * dy;
}

namespace dwb_critics
{
    void RotateBeforeStartCritic::onInit(){
        //get parameters and indicate that we are at the start of the navigation.
        ROS_WARN_NAMED("RotateBeforeStart", "Basliyoz haaa");
        start_yaw_tolerance_ = nav_2d_utils::searchAndGetParam(critic_nh_, "start_yaw_tolerance", 0.5);
        start_yaw_tolerance_sq = start_yaw_tolerance_ * start_yaw_tolerance_;
        is_start_ = true;
    }

    void RotateBeforeStartCritic::reset(){
        ROS_WARN_NAMED("RotateBeforeStart", "reset works");
        is_start_ = true;
    }

    bool RotateBeforeStartCritic::prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel, const geometry_msgs::Pose2D& goal, const nav_2d_msgs::Path2D& global_plan){
        //This method gets the new global plan and the current information about the robot on a global plan update.
        current_pose_ = pose;
        if (is_start_){
            start_pose_ = global_plan.poses[0];
            start_pose_2 = global_plan.poses[10];
            start_yaw_ = atan2(start_pose_2.y-start_pose_.y, start_pose_2.x-start_pose_.x);
            global_plan_ = global_plan;
            current_goal_ = goal;

            if (hypot_sq(pose.x - goal.x, pose.y - goal.y) <= start_yaw_tolerance_sq
                || fabs(angles::shortest_angular_distance(pose.theta, start_yaw_)) < 0.2) {
                    is_start_ = false;
            }
        }
        return true;
    }

    double RotateBeforeStartCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj){

        float difference = fabs(angles::shortest_angular_distance(current_pose_.theta, start_yaw_));
        
        if(is_start_){

            if(difference < start_yaw_tolerance_){
                std::cout << "if/yaw difference: " << difference << std::endl;
                is_start_ = false;
                return 0.0;
            } 
            
            if(traj.velocity.x != 0.0 || traj.velocity.y != 0.0 || (traj.velocity.theta < 0.7 && traj.velocity.theta > -0.7)){
                return 500.0;
            }
        }
        return difference;
    }

}//namespace dwb_critics

#ifndef DWB_CRITICS_TITREK_H
#define DWB_CRITICS_TITREK_H

#include <dwb_local_planner/trajectory_critic.h>
#include <iostream>
#include <vector>
#include <std_msgs/Float64.h>

namespace dwb_critics
{
/**
 * @class RotateBeforeStartCritic
 * @brief Robot point towards to the global plan before starting to move in linear motion everytime we obtain a new goal.
 */
    class TitrekCritic : public dwb_local_planner::TrajectoryCritic
    {
    public:
        void onInit() override;
        double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;
        void reset();
        bool prepare(const geometry_msgs::Pose2D& pose, 
                     const nav_2d_msgs::Twist2D& vel,
                     const geometry_msgs::Pose2D& goal,
                     const nav_2d_msgs::Path2D& global_plan);
        void debrief(const nav_2d_msgs::Twist2D & 	cmd_vel);
    protected:
        geometry_msgs::Pose2D current_pose_;
        geometry_msgs::Pose2D start_pose_;
        geometry_msgs::Pose2D start_pose_2;
        nav_2d_msgs::Path2D global_plan_;
        geometry_msgs::Pose2D current_goal_;
        float yaw_tolerance_;
        std::vector<double> yaws;
        bool oscillation_flag;
        int oscillation_mode_;
    };
}

#endif //DWB_CRITICS_ROTATE_BEFORE_START_H

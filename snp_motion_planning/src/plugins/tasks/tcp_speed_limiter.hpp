#pragma once

#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_time_parameterization/core/trajectory_container.h>
#include <tesseract_environment/environment.h>

namespace snp_motion_planning
{
void limitTCPSpeed(tesseract_environment::Environment::ConstPtr env,
                   tesseract_planning::TrajectoryContainer& trajectory, const std::string& joint_group,
                   const std::string& tcp_frame, const double max_speed)
{
  // Extract objects needed for calculating FK
  tesseract_scene_graph::StateSolver::UPtr state_solver = env->getStateSolver();
  std::vector<std::string> joint_names = env->getJointGroup(joint_group)->getJointNames();

  // Find the adjacent waypoints that require the biggest speed reduction to stay under the max tcp speed
  double strongest_scaling_factor = 1.0;
  for (Eigen::Index i = 1; i < trajectory.size(); i++)
  {
    // Find the previous waypoint position in Cartesian space
    tesseract_scene_graph::SceneState prev_ss = state_solver->getState(joint_names, trajectory.getPosition(i - 1));
    Eigen::Isometry3d prev_pose = prev_ss.link_transforms[tcp_frame];

    // Find the current waypoint position in Cartesian space
    tesseract_scene_graph::SceneState curr_ss = state_solver->getState(joint_names, trajectory.getPosition(i));
    Eigen::Isometry3d curr_pose = curr_ss.link_transforms[tcp_frame];

    // Calculate the average TCP velocity between these waypoints
    double dist_traveled = (curr_pose.translation() - prev_pose.translation()).norm();
    double time_to_travel = trajectory.getTimeFromStart(i) - trajectory.getTimeFromStart(i - 1);
    double original_velocity = dist_traveled / time_to_travel;

    // If the velocity is over the max speed determine the scaling factor and update greatest seen to this point
    if (original_velocity > max_speed)
    {
      double current_needed_scaling_factor = max_speed / original_velocity;
      if (current_needed_scaling_factor < strongest_scaling_factor)
        strongest_scaling_factor = current_needed_scaling_factor;
    }
  }

  // Apply the strongest scaling factor to all trajectory points to maintain a smooth trajectory
  double total_time = 0;
  double prev_time = trajectory.getTimeFromStart(0);
  for (Eigen::Index i = 1; i < trajectory.size(); i++)
  {
    // Calculate new timestamp
    double original_time_diff = trajectory.getTimeFromStart(i) - prev_time;
    double new_time_diff = original_time_diff / strongest_scaling_factor;
    double new_timestamp = total_time + new_time_diff;

    // Scale joint velocity by the scaling factor
    const Eigen::VectorXd joint_vel = trajectory.getVelocity(i) * strongest_scaling_factor;

    // Scale joint acceleartion by the scaling factor squared
    const Eigen::VectorXd joint_acc =
        trajectory.getAcceleration(i) * strongest_scaling_factor * strongest_scaling_factor;

    // Update the previous time
    prev_time = trajectory.getTimeFromStart(i);

    // Update the trajectory container
    trajectory.setData(i, joint_vel, joint_acc, new_timestamp);

    // Update the total running time of the trajectory up to this point
    total_time = new_timestamp;
  }
}

}  // namespace snp_motion_planning

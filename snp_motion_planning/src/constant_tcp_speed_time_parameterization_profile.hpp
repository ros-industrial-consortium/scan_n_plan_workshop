#pragma once

#include <memory>
#include <cmath>

namespace snp_motion_planning
{
struct ConstantTCPSpeedTimeParameterizationProfile
{
  using Ptr = std::shared_ptr<ConstantTCPSpeedTimeParameterizationProfile>;
  using ConstPtr = std::shared_ptr<const ConstantTCPSpeedTimeParameterizationProfile>;

  ConstantTCPSpeedTimeParameterizationProfile() = default;
  ConstantTCPSpeedTimeParameterizationProfile(double max_translational_velocity, double max_rotational_velocity,
                                              double max_translational_acceleration, double max_rotational_acceleration,
                                              bool check_joint_accelerations = false,
                                              double max_velocity_scaling_factor = 1.0,
                                              double max_acceleration_scaling_factor = 1.0)
    : max_translational_velocity(max_translational_velocity)
    , max_rotational_velocity(max_rotational_velocity)
    , max_translational_acceleration(max_translational_acceleration)
    , max_rotational_acceleration(max_rotational_acceleration)
    , check_joint_accelerations(check_joint_accelerations)
    , max_velocity_scaling_factor(max_velocity_scaling_factor)
    , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
  {
  }

  double max_translational_velocity;
  double max_rotational_velocity;
  double max_translational_acceleration;
  double max_rotational_acceleration;

  /** @brief Flag to enable checking of planned joint accelerations against joint acceleration limits
   *  @details Checking joint accelerations is optional because often joint accleration limits for manipulators are not
   * provided by manufacturers and their values depend on the run-time configuraion of the robot (e.g., payload, etc.)
   */
  bool check_joint_accelerations;

  double max_velocity_scaling_factor;
  double max_acceleration_scaling_factor;
};

}  // namespace snp_motion_planning

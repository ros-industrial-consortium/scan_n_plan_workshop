#pragma once

#include <memory>

namespace snp_motion_planning
{
struct CartesianTimeParameterizationProfile
{
  using Ptr = std::shared_ptr<CartesianTimeParameterizationProfile>;
  using ConstPtr = std::shared_ptr<const CartesianTimeParameterizationProfile>;

  CartesianTimeParameterizationProfile() = default;
  CartesianTimeParameterizationProfile(double max_translational_velocity, double max_rotational_velocity,
                                       double max_translational_acceleration, double max_rotational_acceleration,
                                       double max_velocity_scaling_factor = 1.0,
                                       double max_acceleration_scaling_factor = 1.0)
    : max_translational_velocity(max_translational_velocity)
    , max_rotational_velocity(max_rotational_velocity)
    , max_translational_acceleration(max_translational_acceleration)
    , max_rotational_acceleration(max_rotational_acceleration)
    , max_velocity_scaling_factor(max_velocity_scaling_factor)
    , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
  {
  }

  double max_translational_velocity;
  double max_rotational_velocity;
  double max_translational_acceleration;
  double max_rotational_acceleration;
  double max_velocity_scaling_factor;
  double max_acceleration_scaling_factor;
};

}  // namespace snp_motion_planning

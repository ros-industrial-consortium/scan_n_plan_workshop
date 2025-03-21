#pragma once

#include <memory>
#include <cmath>
#include <string>
#include <tesseract_command_language/profile.h>

static const std::string CONSTANT_TCP_SPEED_TIME_PARAM_TASK_NAME = "ConstantTCPSpeedTimeParameterizationTask";

namespace snp_motion_planning
{
struct ConstantTCPSpeedTimeParameterizationProfile : public tesseract_planning::Profile
{
  using Ptr = std::shared_ptr<ConstantTCPSpeedTimeParameterizationProfile>;
  using ConstPtr = std::shared_ptr<const ConstantTCPSpeedTimeParameterizationProfile>;

  ConstantTCPSpeedTimeParameterizationProfile();
  ConstantTCPSpeedTimeParameterizationProfile(double max_translational_velocity_, double max_rotational_velocity_,
                                              double max_translational_acceleration_,
                                              double max_rotational_acceleration_,
                                              double max_velocity_scaling_factor_ = 1.0,
                                              double max_acceleration_scaling_factor_ = 1.0);

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  double max_translational_velocity;
  double max_rotational_velocity;
  double max_translational_acceleration;
  double max_rotational_acceleration;
  double max_velocity_scaling_factor;
  double max_acceleration_scaling_factor;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

}  // namespace snp_motion_planning

BOOST_CLASS_EXPORT_KEY(snp_motion_planning::ConstantTCPSpeedTimeParameterizationProfile)

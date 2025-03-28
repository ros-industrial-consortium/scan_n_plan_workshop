#pragma once

#include <memory>
#include <string>
#include <tesseract_command_language/profile.h>

static const std::string KINEMATIC_LIMITS_CHECK_TASK_NAME = "KinematicLimitsCheckTask";

namespace snp_motion_planning
{
struct KinematicLimitsCheckProfile : public tesseract_planning::Profile
{
  using Ptr = std::shared_ptr<KinematicLimitsCheckProfile>;
  using ConstPtr = std::shared_ptr<const KinematicLimitsCheckProfile>;

  KinematicLimitsCheckProfile(bool check_position_ = true, bool check_velocity_ = true,
                              bool check_acceleration_ = true);

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  bool check_position;
  bool check_velocity;
  bool check_acceleration;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

}  // namespace snp_motion_planning

BOOST_CLASS_EXPORT_KEY(snp_motion_planning::KinematicLimitsCheckProfile)

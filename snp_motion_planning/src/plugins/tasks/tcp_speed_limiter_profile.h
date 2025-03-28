#pragma once

#include <memory>
#include <string>
#include <tesseract_command_language/profile.h>

static const std::string TCP_SPEED_LIMITER_TASK_NAME = "TCPSpeedLimiterTask";

namespace snp_motion_planning
{
struct TCPSpeedLimiterProfile : public tesseract_planning::Profile
{
  using Ptr = std::shared_ptr<TCPSpeedLimiterProfile>;
  using ConstPtr = std::shared_ptr<const TCPSpeedLimiterProfile>;

  TCPSpeedLimiterProfile();
  TCPSpeedLimiterProfile(double max_speed_);

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  double max_speed;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

}  // namespace snp_motion_planning

BOOST_CLASS_EXPORT_KEY(snp_motion_planning::TCPSpeedLimiterProfile)

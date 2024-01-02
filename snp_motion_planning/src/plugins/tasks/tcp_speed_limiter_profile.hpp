#pragma once

#include <memory>
#include <string>

static const std::string TCP_SPEED_LIMITER_TASK_NAME = "TCPSpeedLimiterTask";

namespace snp_motion_planning
{
struct TCPSpeedLimiterProfile
{
  using Ptr = std::shared_ptr<TCPSpeedLimiterProfile>;
  using ConstPtr = std::shared_ptr<const TCPSpeedLimiterProfile>;

  TCPSpeedLimiterProfile() = default;

  TCPSpeedLimiterProfile(double max_speed_) : max_speed(max_speed_)

  {
  }
  double max_speed;
};

}  // namespace snp_motion_planning
#include "tcp_speed_limiter_profile.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <typeindex>

namespace snp_motion_planning
{

TCPSpeedLimiterProfile::TCPSpeedLimiterProfile() : Profile(TCPSpeedLimiterProfile::getStaticKey())
{
}

TCPSpeedLimiterProfile::TCPSpeedLimiterProfile(double max_speed_)
  : Profile(TCPSpeedLimiterProfile::getStaticKey()), max_speed(max_speed_)
{
}

std::size_t TCPSpeedLimiterProfile::getStaticKey()
{
  return std::type_index(typeid(TCPSpeedLimiterProfile)).hash_code();
}

template <class Archive>
void TCPSpeedLimiterProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(max_speed);
}

}  // namespace snp_motion_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(snp_motion_planning::TCPSpeedLimiterProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(snp_motion_planning::TCPSpeedLimiterProfile)

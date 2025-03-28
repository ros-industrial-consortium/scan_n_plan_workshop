#include "kinematic_limits_check_profile.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <typeindex>

namespace snp_motion_planning
{
KinematicLimitsCheckProfile::KinematicLimitsCheckProfile(bool check_position_, bool check_velocity_,
                                                         bool check_acceleration_)
  : Profile(KinematicLimitsCheckProfile::getStaticKey())
  , check_position(check_position_)
  , check_velocity(check_velocity_)
  , check_acceleration(check_acceleration_)
{
}

std::size_t KinematicLimitsCheckProfile::getStaticKey()
{
  return std::type_index(typeid(KinematicLimitsCheckProfile)).hash_code();
}

template <class Archive>
void KinematicLimitsCheckProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(check_position);
  ar& BOOST_SERIALIZATION_NVP(check_velocity);
  ar& BOOST_SERIALIZATION_NVP(check_acceleration);
}
}  // namespace snp_motion_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(snp_motion_planning::KinematicLimitsCheckProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(snp_motion_planning::KinematicLimitsCheckProfile)

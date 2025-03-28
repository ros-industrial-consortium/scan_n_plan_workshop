#include "constant_tcp_speed_time_parameterization_profile.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <typeindex>

namespace snp_motion_planning
{
ConstantTCPSpeedTimeParameterizationProfile::ConstantTCPSpeedTimeParameterizationProfile()
  : Profile(ConstantTCPSpeedTimeParameterizationProfile::getStaticKey())
{
}
ConstantTCPSpeedTimeParameterizationProfile::ConstantTCPSpeedTimeParameterizationProfile(
    double max_translational_velocity_, double max_rotational_velocity_, double max_translational_acceleration_,
    double max_rotational_acceleration_, double max_velocity_scaling_factor_, double max_acceleration_scaling_factor_)
  : Profile(ConstantTCPSpeedTimeParameterizationProfile::getStaticKey())
  , max_translational_velocity(max_translational_velocity_)
  , max_rotational_velocity(max_rotational_velocity_)
  , max_translational_acceleration(max_translational_acceleration_)
  , max_rotational_acceleration(max_rotational_acceleration_)
  , max_velocity_scaling_factor(max_velocity_scaling_factor_)
  , max_acceleration_scaling_factor(max_acceleration_scaling_factor_)
{
}

std::size_t ConstantTCPSpeedTimeParameterizationProfile::getStaticKey()
{
  return std::type_index(typeid(ConstantTCPSpeedTimeParameterizationProfile)).hash_code();
}

template <class Archive>
void ConstantTCPSpeedTimeParameterizationProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(max_translational_velocity);
  ar& BOOST_SERIALIZATION_NVP(max_rotational_velocity);
  ar& BOOST_SERIALIZATION_NVP(max_translational_acceleration);
  ar& BOOST_SERIALIZATION_NVP(max_rotational_acceleration);
  ar& BOOST_SERIALIZATION_NVP(max_velocity_scaling_factor);
  ar& BOOST_SERIALIZATION_NVP(max_acceleration_scaling_factor);
}
}  // namespace snp_motion_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(snp_motion_planning::ConstantTCPSpeedTimeParameterizationProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(snp_motion_planning::ConstantTCPSpeedTimeParameterizationProfile)

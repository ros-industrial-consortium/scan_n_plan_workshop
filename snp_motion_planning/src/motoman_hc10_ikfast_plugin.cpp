/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/core/demangle.hpp>
#include <Eigen/Geometry>
#include <tesseract_kinematics/ikfast/impl/ikfast_inv_kin.hpp>
#include <tesseract_kinematics/core/kinematics_plugin_factory.h>
#include "motoman_hc10_ikfast_solver.hpp"
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_kinematics
{
template <typename T>
T get(const YAML::Node& node, const std::string& key)
{
  try
  {
    return node[key].as<T>();
  }
  catch (const YAML::Exception&)
  {
    std::stringstream ss;
    ss << "Failed to extract '" << key << "' from configuration with type '" << boost::core::demangle(typeid(T).name());
    throw std::runtime_error(ss.str());
  }
}

class IKFastInvKinFactory : public InvKinFactory
{
public:
  virtual InverseKinematics::UPtr create(const std::string& solver_name,
                                         const tesseract_scene_graph::SceneGraph& scene_graph,
                                         const tesseract_scene_graph::SceneState& /*scene_state*/,
                                         const KinematicsPluginFactory& /*plugin_factory*/,
                                         const YAML::Node& config) const
  {
    auto base_link = get<std::string>(config, "base_link");
    auto tip_link = get<std::string>(config, "tip_link");
    std::size_t n_joints = get<std::size_t>(config, "n_joints");

    // Get the free joint states
    std::vector<std::vector<double>> free_joint_states;
    {
      const YAML::Node& free_joint_states_node = config["free_joint_states"];
      for (const YAML::Node& n : free_joint_states_node)
      {
        auto states = n.as<std::vector<double>>();
        free_joint_states.insert(free_joint_states.begin(), states);
      }
    }

    // Get the active joints in between the base link and tip link
    tesseract_scene_graph::ShortestPath path = scene_graph.getShortestPath(base_link, tip_link);
    if (path.active_joints.size() != n_joints)
    {
      std::stringstream ss;
      ss << "Kinematic chain between '" << base_link << "' and '" << tip_link << "' contains "
         << path.active_joints.size() << " joints, but only " << n_joints << " can be used";
      throw std::runtime_error(ss.str());
    }

    return std::make_unique<IKFastInvKin>(base_link, tip_link, path.active_joints, solver_name, free_joint_states);
  }
};

}  // namespace tesseract_kinematics

TESSERACT_ADD_PLUGIN(tesseract_kinematics::IKFastInvKinFactory, MotomanHC10InvKinFactory)

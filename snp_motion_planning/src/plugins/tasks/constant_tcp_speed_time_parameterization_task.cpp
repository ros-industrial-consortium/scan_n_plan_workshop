#include "constant_tcp_speed_time_parameterization.hpp"
#include "constant_tcp_speed_time_parameterization_profile.h"

#include <tesseract_task_composer/core/task_composer_plugin_factory_utils.h>

#include <tesseract_common/macros.h>
#include <boost/serialization/string.hpp>

#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory_utils.h>
#include <tesseract_task_composer/core/task_composer_task_plugin_factory.h>

// Requried
const std::string INOUT_PROGRAM_PORT = "program";
const std::string INPUT_ENVIRONMENT_PORT = "environment";
const std::string INPUT_PROFILES_PORT = "profiles";

namespace snp_motion_planning
{
class ConstantTCPSpeedTimeParameterizationTask : public tesseract_planning::TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<ConstantTCPSpeedTimeParameterizationTask>;
  using ConstPtr = std::shared_ptr<const ConstantTCPSpeedTimeParameterizationTask>;
  using UPtr = std::unique_ptr<ConstantTCPSpeedTimeParameterizationTask>;
  using ConstUPtr = std::unique_ptr<const ConstantTCPSpeedTimeParameterizationTask>;

  static tesseract_planning::TaskComposerNodePorts ports()
  {
    tesseract_planning::TaskComposerNodePorts ports;
    ports.input_required[INOUT_PROGRAM_PORT] = tesseract_planning::TaskComposerNodePorts::SINGLE;
    ports.input_required[INPUT_ENVIRONMENT_PORT] = tesseract_planning::TaskComposerNodePorts::SINGLE;
    ports.input_required[INPUT_PROFILES_PORT] = tesseract_planning::TaskComposerNodePorts::SINGLE;
    ports.output_required[INOUT_PROGRAM_PORT] = tesseract_planning::TaskComposerNodePorts::SINGLE;
    return ports;
  }

  ConstantTCPSpeedTimeParameterizationTask()
    : tesseract_planning::TaskComposerTask(CONSTANT_TCP_SPEED_TIME_PARAM_TASK_NAME, ports(), true)
  {
  }

  explicit ConstantTCPSpeedTimeParameterizationTask(std::string name, std::string input_program_key,
                                                    std::string input_environment_key, std::string input_profiles_key,
                                                    std::string output_program_key, bool is_conditional = true)
    : tesseract_planning::TaskComposerTask(std::move(name), ports(), is_conditional)
  {
    input_keys_.add(INOUT_PROGRAM_PORT, std::move(input_program_key));
    input_keys_.add(INPUT_ENVIRONMENT_PORT, std::move(input_environment_key));
    input_keys_.add(INPUT_PROFILES_PORT, std::move(input_profiles_key));
    output_keys_.add(INOUT_PROGRAM_PORT, std::move(output_program_key));
    validatePorts();
  }

  explicit ConstantTCPSpeedTimeParameterizationTask(
      std::string name, const YAML::Node& config,
      const tesseract_planning::TaskComposerPluginFactory& /*plugin_factory*/)
    : tesseract_planning::TaskComposerTask(std::move(name), ports(), config)
  {
  }

  ~ConstantTCPSpeedTimeParameterizationTask() override = default;
  ConstantTCPSpeedTimeParameterizationTask(const ConstantTCPSpeedTimeParameterizationTask&) = delete;
  ConstantTCPSpeedTimeParameterizationTask& operator=(const ConstantTCPSpeedTimeParameterizationTask&) = delete;
  ConstantTCPSpeedTimeParameterizationTask(ConstantTCPSpeedTimeParameterization&&) = delete;
  ConstantTCPSpeedTimeParameterizationTask& operator=(ConstantTCPSpeedTimeParameterizationTask&&) = delete;

  bool operator==(const ConstantTCPSpeedTimeParameterizationTask& rhs) const
  {
    return tesseract_planning::TaskComposerTask::operator==(rhs);
  }
  bool operator!=(const ConstantTCPSpeedTimeParameterizationTask& rhs) const
  {
    return !operator==(rhs);
  }

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;
  tesseract_planning::TaskComposerNodeInfo runImpl(tesseract_planning::TaskComposerContext& context,
                                                   OptionalTaskComposerExecutor /*executor*/) const override final
  {
    tesseract_planning::TaskComposerNodeInfo info(*this);
    info.return_value = 0;
    info.status_code = 0;

    if (context.isAborted())
    {
      info.status_message = "Aborted";
      return info;
    }

    // --------------------
    // Check that inputs are valid
    // --------------------
    auto env_poly = getData(*context.data_storage, INPUT_ENVIRONMENT_PORT);
    if (env_poly.getType() != std::type_index(typeid(std::shared_ptr<const tesseract_environment::Environment>)))
    {
      info.color = "red";
      info.status_message = "Input data '" + input_keys_.get(INPUT_ENVIRONMENT_PORT) + "' is not correct type";
      return info;
    }

    auto env = env_poly.as<std::shared_ptr<const tesseract_environment::Environment>>();

    auto input_data_poly = getData(*context.data_storage, INOUT_PROGRAM_PORT);
    if (input_data_poly.getType() != std::type_index(typeid(tesseract_planning::CompositeInstruction)))
    {
      info.color = "red";
      info.status_message = "Input to KinematicLimitsCheckTask must be a composite instruction";
      return info;
    }

    // Get Composite Profile
    auto profiles = getData(*context.data_storage, INPUT_PROFILES_PORT)
                        .as<std::shared_ptr<tesseract_planning::ProfileDictionary>>();
    auto& ci = input_data_poly.as<tesseract_planning::CompositeInstruction>();
    auto cur_composite_profile = tesseract_planning::getProfile<ConstantTCPSpeedTimeParameterizationProfile>(
        ns_, ci.getProfile(ns_), *profiles, std::make_shared<ConstantTCPSpeedTimeParameterizationProfile>());

    const tesseract_common::ManipulatorInfo& manip_info = ci.getManipulatorInfo();

    // Create data structures for checking for plan profile overrides
    auto flattened = ci.flatten(tesseract_planning::moveFilter);
    if (flattened.empty())
    {
      info.color = "yellow";
      info.status_message = "Cartesian time parameterization found no MoveInstructions to process";
      info.status_code = 1;
      info.return_value = 1;
      return info;
    }

    // Solve using parameters
    tesseract_planning::TrajectoryContainer::Ptr trajectory =
        std::make_shared<tesseract_planning::InstructionsTrajectory>(ci);

    ConstantTCPSpeedTimeParameterization solver(
        env, manip_info.manipulator, manip_info.tcp_frame, cur_composite_profile->max_translational_velocity,
        cur_composite_profile->max_rotational_velocity, cur_composite_profile->max_translational_acceleration,
        cur_composite_profile->max_rotational_acceleration);

    if (!solver.compute(*trajectory, cur_composite_profile->max_velocity_scaling_factor,
                        cur_composite_profile->max_acceleration_scaling_factor))
    {
      info.color = "red";
      info.status_message =
          "Failed to perform constant TCP speed time parameterization for process input: " + ci.getDescription();
      return info;
    }

    setData(*context.data_storage, INOUT_PROGRAM_PORT, ci);

    info.color = "green";
    info.status_message = "Constant TCP speed time parameterization succeeded";
    info.status_code = 1;
    info.return_value = 1;
    return info;
  }

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(tesseract_planning::TaskComposerTask);
  }
};

using ConstantTCPSpeedTimeParameterizationTaskFactory =
    tesseract_planning::TaskComposerTaskFactory<ConstantTCPSpeedTimeParameterizationTask>;

}  // namespace snp_motion_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(snp_motion_planning::ConstantTCPSpeedTimeParameterizationTask)
BOOST_CLASS_EXPORT_IMPLEMENT(snp_motion_planning::ConstantTCPSpeedTimeParameterizationTask)

TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(snp_motion_planning::ConstantTCPSpeedTimeParameterizationTaskFactory,
                                        ConstantTCPSpeedTimeParameterizationTaskFactory)

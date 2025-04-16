#include "tcp_speed_limiter_profile.h"
#include "tcp_speed_limiter.hpp"

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
class TCPSpeedLimiterTask : public tesseract_planning::TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<TCPSpeedLimiterTask>;
  using ConstPtr = std::shared_ptr<const TCPSpeedLimiterTask>;
  using UPtr = std::unique_ptr<TCPSpeedLimiterTask>;
  using ConstUPtr = std::unique_ptr<const TCPSpeedLimiterTask>;

  static tesseract_planning::TaskComposerNodePorts ports()
  {
    tesseract_planning::TaskComposerNodePorts ports;
    ports.input_required[INOUT_PROGRAM_PORT] = tesseract_planning::TaskComposerNodePorts::SINGLE;
    ports.input_required[INPUT_ENVIRONMENT_PORT] = tesseract_planning::TaskComposerNodePorts::SINGLE;
    ports.input_required[INPUT_PROFILES_PORT] = tesseract_planning::TaskComposerNodePorts::SINGLE;
    ports.output_required[INOUT_PROGRAM_PORT] = tesseract_planning::TaskComposerNodePorts::SINGLE;
    return ports;
  }

  TCPSpeedLimiterTask() : tesseract_planning::TaskComposerTask(TCP_SPEED_LIMITER_TASK_NAME, ports(), true)
  {
  }

  explicit TCPSpeedLimiterTask(std::string name, std::string input_program_key, std::string input_environment_key,
                               std::string input_profiles_key, std::string output_program_key,
                               bool is_conditional = true)
    : tesseract_planning::TaskComposerTask(std::move(name), ports(), is_conditional)
  {
    input_keys_.add(INOUT_PROGRAM_PORT, std::move(input_program_key));
    input_keys_.add(INPUT_ENVIRONMENT_PORT, std::move(input_environment_key));
    input_keys_.add(INPUT_PROFILES_PORT, std::move(input_profiles_key));
    output_keys_.add(INOUT_PROGRAM_PORT, std::move(output_program_key));
    validatePorts();
  }

  explicit TCPSpeedLimiterTask(std::string name, const YAML::Node& config,
                               const tesseract_planning::TaskComposerPluginFactory& /*plugin_factory*/)
    : tesseract_planning::TaskComposerTask(std::move(name), ports(), config)
  {
    validatePorts();
  }

  ~TCPSpeedLimiterTask() override = default;
  TCPSpeedLimiterTask(const TCPSpeedLimiterTask&) = delete;
  TCPSpeedLimiterTask& operator=(const TCPSpeedLimiterTask&) = delete;
  TCPSpeedLimiterTask(TCPSpeedLimiterTask&&) = delete;
  TCPSpeedLimiterTask& operator=(TCPSpeedLimiterTask&&) = delete;

  bool operator==(const TCPSpeedLimiterTask& rhs) const
  {
    return (tesseract_planning::TaskComposerTask::operator==(rhs));
  }
  bool operator!=(const TCPSpeedLimiterTask& rhs) const
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

    // Get the environment input
    auto env_poly = getData(*context.data_storage, INPUT_ENVIRONMENT_PORT);
    if (env_poly.getType() != std::type_index(typeid(std::shared_ptr<const tesseract_environment::Environment>)))
    {
      info.color = "red";
      info.status_message =
          "Input data " + input_keys_.get(INPUT_ENVIRONMENT_PORT) + " is missing or of incorrect type";
      return info;
    }
    auto env = env_poly.as<std::shared_ptr<const tesseract_environment::Environment>>();

    // Composite instruction input
    auto input_data_poly = getData(*context.data_storage, INOUT_PROGRAM_PORT);
    if (input_data_poly.getType() != std::type_index(typeid(tesseract_planning::CompositeInstruction)))
    {
      info.color = "red";
      info.status_message = "Input data " + input_keys_.get(INOUT_PROGRAM_PORT) + " is missing or of incorrect type";
      return info;
    }
    auto& ci = input_data_poly.as<tesseract_planning::CompositeInstruction>();
    const tesseract_common::ManipulatorInfo& manip_info = ci.getManipulatorInfo();

    // Get the profile dictionary input
    auto profiles_poly = getData(*context.data_storage, INPUT_PROFILES_PORT);
    if (profiles_poly.isNull() ||
        profiles_poly.getType() != std::type_index(typeid(std::shared_ptr<tesseract_planning::ProfileDictionary>)))
    {
      info.color = "red";
      info.status_message = "Input data " + input_keys_.get(INPUT_PROFILES_PORT) + " is missing or of incorrect type";
      return info;
    }
    auto profiles = profiles_poly.as<std::shared_ptr<tesseract_planning::ProfileDictionary>>();

    // Get Composite Profile
    std::string profile = ci.getProfile(ns_);
    // profile = tesseract_planning::getProfileString(name_, profile, problem.composite_profile_remapping);
    auto cur_composite_profile = tesseract_planning::getProfile<TCPSpeedLimiterProfile>(
        name_, profile, *profiles, std::make_shared<TCPSpeedLimiterProfile>());

    // Create data structures for checking for plan profile overrides
    auto flattened = ci.flatten(tesseract_planning::moveFilter);
    if (flattened.empty())
    {
      info.color = "yellow";
      info.status_message = "TCP speed limiter task found no MoveInstructions to process";
      info.status_code = 1;
      info.return_value = 1;
      return info;
    }

    // Solve using parameters
    tesseract_planning::TrajectoryContainer::Ptr trajectory =
        std::make_shared<tesseract_planning::InstructionsTrajectory>(ci);
    const std::string joint_group = env->getJointGroup(manip_info.manipulator)->getName();

    try
    {
      limitTCPSpeed(env, *trajectory, joint_group, manip_info.tcp_frame, cur_composite_profile->max_speed);
    }
    catch (const std::exception& ex)
    {
      info.status_message = ex.what();
      return info;
    }

    setData(*context.data_storage, INOUT_PROGRAM_PORT, input_data_poly);

    info.color = "green";
    info.status_message = "TCP speed limiter task succeeded";
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

using TCPSpeedLimiterTaskFactory = tesseract_planning::TaskComposerTaskFactory<TCPSpeedLimiterTask>;

}  // namespace snp_motion_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(snp_motion_planning::TCPSpeedLimiterTask)
BOOST_CLASS_EXPORT_IMPLEMENT(snp_motion_planning::TCPSpeedLimiterTask)

TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(snp_motion_planning::TCPSpeedLimiterTaskFactory, TCPSpeedLimiterTaskFactory)

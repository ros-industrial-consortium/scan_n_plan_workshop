#include "tcp_speed_limiter_profile.hpp"
#include "tcp_speed_limiter.hpp"

#include <tesseract_task_composer/core/task_composer_plugin_factory_utils.h>

#include <tesseract_common/macros.h>
#include <console_bridge/console.h>
#include <boost/serialization/string.hpp>

#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>

#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory_utils.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>

namespace snp_motion_planning
{
class TCPSpeedLimiterTask : public tesseract_planning::TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<TCPSpeedLimiterTask>;
  using ConstPtr = std::shared_ptr<const TCPSpeedLimiterTask>;
  using UPtr = std::unique_ptr<TCPSpeedLimiterTask>;
  using ConstUPtr = std::unique_ptr<const TCPSpeedLimiterTask>;

  TCPSpeedLimiterTask() : tesseract_planning::TaskComposerTask(TCP_SPEED_LIMITER_TASK_NAME, true)
  {
  }

  explicit TCPSpeedLimiterTask(std::string name, std::string input_key, std::string output_key,
                               bool is_conditional = true)
    : tesseract_planning::TaskComposerTask(std::move(name), is_conditional)
  {
    input_keys_.push_back(std::move(input_key));
    output_keys_.push_back(std::move(output_key));
  }

  explicit TCPSpeedLimiterTask(std::string name, const YAML::Node& config,
                               const tesseract_planning::TaskComposerPluginFactory& /*plugin_factory*/)
    : tesseract_planning::TaskComposerTask(std::move(name), config)
  {
    if (input_keys_.empty())
      throw std::runtime_error("TCPSpeedLimiterTask, config missing 'inputs' entry");

    if (input_keys_.size() > 1)
      throw std::runtime_error("TCPSpeedLimiterTask, config 'inputs' entry currently only "
                               "supports "
                               "one input key");

    if (output_keys_.empty())
      throw std::runtime_error("TCPSpeedLimiterTask, config missing 'outputs' entry");

    if (output_keys_.size() > 1)
      throw std::runtime_error("TCPSpeedLimiterTask, config 'outputs' entry currently only "
                               "supports "
                               "one output key");
  }

  ~TCPSpeedLimiterTask() override = default;
  TCPSpeedLimiterTask(const TCPSpeedLimiterTask&) = delete;
  TCPSpeedLimiterTask& operator=(const TCPSpeedLimiterTask&) = delete;
  TCPSpeedLimiterTask(TCPSpeedLimiterTask&&) = delete;
  TCPSpeedLimiterTask& operator=(TCPSpeedLimiterTask&&) = delete;

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;
  tesseract_planning::TaskComposerNodeInfo::UPtr runImpl(tesseract_planning::TaskComposerContext& context,
                                                         OptionalTaskComposerExecutor /*executor*/) const override final
  {
    auto info = std::make_unique<tesseract_planning::TaskComposerNodeInfo>(*this);
    info->return_value = 0;

    // Get the problem
    auto& problem = dynamic_cast<tesseract_planning::PlanningTaskComposerProblem&>(*context.problem);

    // --------------------
    // Check that inputs are valid
    // --------------------
    auto input_data_poly = context.data_storage->getData(input_keys_[0]);
    auto& ci = input_data_poly.as<tesseract_planning::CompositeInstruction>();
    const tesseract_common::ManipulatorInfo& manip_info = ci.getManipulatorInfo();

    // Get Composite Profile
    std::string profile = ci.getProfile();
    profile = tesseract_planning::getProfileString(name_, profile, problem.composite_profile_remapping);
    auto cur_composite_profile = tesseract_planning::getProfile<TCPSpeedLimiterProfile>(
        name_, profile, *problem.profiles, std::make_shared<TCPSpeedLimiterProfile>());
    cur_composite_profile =
        tesseract_planning::applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

    // Create data structures for checking for plan profile overrides
    auto flattened = ci.flatten(tesseract_planning::moveFilter);
    if (flattened.empty())
    {
      info->message = "TCP speed limiter task found no MoveInstructions to process";
      info->return_value = 1;
      return info;
    }

    // Solve using parameters
    tesseract_planning::TrajectoryContainer::Ptr trajectory =
        std::make_shared<tesseract_planning::InstructionsTrajectory>(ci);
    const std::string joint_group = problem.env->getJointGroup(manip_info.manipulator)->getName();

    try
    {
      limitTCPSpeed(problem.env, *trajectory, joint_group, manip_info.tcp_frame, cur_composite_profile->max_speed);
    }
    catch (const std::exception& ex)
    {
      info->message = ex.what();
      return info;
    }

    info->color = "green";
    info->message = "TCP speed limiter task succeeded";
    context.data_storage->setData(output_keys_[0], input_data_poly);
    info->return_value = 1;
    return info;
  }

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(tesseract_planning::TaskComposerTask);
  }
};

}  // namespace snp_motion_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(snp_motion_planning::TCPSpeedLimiterTask)
BOOST_CLASS_EXPORT_IMPLEMENT(snp_motion_planning::TCPSpeedLimiterTask)

TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(
    tesseract_planning::TaskComposerTaskFactory<snp_motion_planning::TCPSpeedLimiterTask>, TCPSpeedLimiterTaskFactory)

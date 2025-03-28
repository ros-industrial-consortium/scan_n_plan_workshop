#include "kinematic_limits_check_profile.h"

#include <tesseract_task_composer/core/task_composer_plugin_factory_utils.h>

#include <tesseract_common/macros.h>
#include <boost/serialization/string.hpp>

#include <tesseract_common/timer.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>
#include <tesseract_environment/environment.h>

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
class KinematicLimitsCheckTask : public tesseract_planning::TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<KinematicLimitsCheckTask>;
  using ConstPtr = std::shared_ptr<const KinematicLimitsCheckTask>;
  using UPtr = std::unique_ptr<KinematicLimitsCheckTask>;
  using ConstUPtr = std::unique_ptr<const KinematicLimitsCheckTask>;

  KinematicLimitsCheckTask()
    : tesseract_planning::TaskComposerTask(KINEMATIC_LIMITS_CHECK_TASK_NAME, KinematicLimitsCheckTask::ports(), true)
  {
  }

  explicit KinematicLimitsCheckTask(std::string name, std::string input_program_key, std::string input_environment_key,
                                    std::string input_profiles_key, std::string output_program_key,
                                    bool is_conditional = true)
    : tesseract_planning::TaskComposerTask(std::move(name), KinematicLimitsCheckTask::ports(), is_conditional)
  {
    input_keys_.add(INOUT_PROGRAM_PORT, std::move(input_program_key));
    input_keys_.add(INPUT_ENVIRONMENT_PORT, std::move(input_environment_key));
    input_keys_.add(INPUT_PROFILES_PORT, std::move(input_profiles_key));
    output_keys_.add(INOUT_PROGRAM_PORT, std::move(output_program_key));
    validatePorts();
  }

  explicit KinematicLimitsCheckTask(std::string name, const YAML::Node& config,
                                    const tesseract_planning::TaskComposerPluginFactory& /*plugin_factory*/)
    : tesseract_planning::TaskComposerTask(std::move(name), KinematicLimitsCheckTask::ports(), config)
  {
  }

  ~KinematicLimitsCheckTask() override = default;
  KinematicLimitsCheckTask(const KinematicLimitsCheckTask&) = delete;
  KinematicLimitsCheckTask& operator=(const KinematicLimitsCheckTask&) = delete;
  KinematicLimitsCheckTask(KinematicLimitsCheckTask&&) = delete;
  KinematicLimitsCheckTask& operator=(KinematicLimitsCheckTask&&) = delete;

  static tesseract_planning::TaskComposerNodePorts ports()
  {
    tesseract_planning::TaskComposerNodePorts ports;
    ports.input_required["program"] = tesseract_planning::TaskComposerNodePorts::SINGLE;
    ports.input_required["environment"] = tesseract_planning::TaskComposerNodePorts::SINGLE;
    ports.input_required["profiles"] = tesseract_planning::TaskComposerNodePorts::SINGLE;
    return ports;
  }

  bool operator==(const KinematicLimitsCheckTask& rhs) const
  {
    bool equal = true;
    equal &= tesseract_planning::TaskComposerTask::operator==(rhs);
    return equal;
  }
  bool operator!=(const KinematicLimitsCheckTask& rhs) const
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
    auto cur_composite_profile = tesseract_planning::getProfile<KinematicLimitsCheckProfile>(
        ns_, ci.getProfile(ns_), *profiles, std::make_shared<KinematicLimitsCheckProfile>());

    const tesseract_common::ManipulatorInfo& manip_info = ci.getManipulatorInfo();

    // Create data structures for checking for plan profile overrides
    auto flattened = ci.flatten(tesseract_planning::moveFilter);
    if (flattened.empty())
    {
      info.color = "yellow";
      info.status_message = "Kinematic limits check found no MoveInstructions to process";
      info.status_code = 1;
      info.return_value = 1;
      return info;
    }

    // Wrap the composite instruction in a trajectory container
    tesseract_planning::TrajectoryContainer::Ptr trajectory =
        std::make_shared<tesseract_planning::InstructionsTrajectory>(ci);

    // Extract the motion group
    tesseract_kinematics::JointGroup::ConstPtr motion_group = env->getJointGroup(manip_info.manipulator);
    const auto limits = motion_group->getLimits();

    // Check the trajectory limits
    for (Eigen::Index i = 0; i < trajectory->size(); ++i)
    {
      const Eigen::VectorXd& joint_pos = trajectory->getPosition(i);
      const Eigen::VectorXd& joint_vel = trajectory->getVelocity(i);
      const Eigen::VectorXd& joint_acc = trajectory->getAcceleration(i);

      if (cur_composite_profile->check_position)
      {
        if (!tesseract_common::satisfiesLimits<double>(joint_pos, limits.joint_limits))
        {
          std::stringstream ss;
          ss << "Joint position limit violation(s) at waypoint " << i;
          info.color = "red";
          info.status_message = ss.str();
          return info;
        }
      }

      if (cur_composite_profile->check_velocity)
      {
        // Check for joint velocity limit violations
        if (!tesseract_common::satisfiesLimits<double>(joint_vel, limits.velocity_limits))
        {
          Eigen::ArrayXd capacity = 100.0 * joint_vel.array().abs() / limits.velocity_limits.col(1).array();
          std::stringstream ss;
          ss << "Joint velocity limit violation(s) at waypoint " << i << ": "
             << capacity.transpose().format(Eigen::IOFormat(4, 0, " ", "\n", "[", "]")) << " (%% capacity)";

          info.color = "red";
          info.status_message = ss.str();
          return info;
        }
      }

      if (cur_composite_profile->check_acceleration)
      {
        // Check for joint velocity acceleration limit violations
        if (!tesseract_common::satisfiesLimits<double>(joint_acc, limits.acceleration_limits))
        {
          Eigen::ArrayXd capacity = 100.0 * joint_acc.array().abs() / limits.acceleration_limits.col(1).array();
          std::stringstream ss;
          ss << "Joint acceleration limit violation(s) at waypoint " << i << ": "
             << capacity.transpose().format(Eigen::IOFormat(4, 0, " ", "\n", "[", "]")) << " (%% capacity)";

          info.color = "red";
          info.status_message = ss.str();
          return info;
        }
      }
    }

    info.color = "green";
    info.status_message = "Kinematic limits check succeeded";
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

using KinematicLimitsCheckTaskFactory = tesseract_planning::TaskComposerTaskFactory<KinematicLimitsCheckTask>;

}  // namespace snp_motion_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(snp_motion_planning::KinematicLimitsCheckTask)
BOOST_CLASS_EXPORT_IMPLEMENT(snp_motion_planning::KinematicLimitsCheckTask)

TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(snp_motion_planning::KinematicLimitsCheckTaskFactory,
                                        KinematicLimitsCheckTaskFactory)

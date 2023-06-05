#pragma once

#include "kinematic_limits_check_profile.hpp"

#include <tesseract_common/timer.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_process_managers/core/task_generator.h>
#include <tesseract_process_managers/core/utils.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils/filter_functions.h>
#include <tesseract_command_language/utils/flatten_utils.h>
#include <tesseract_time_parameterization/instructions_trajectory.h>

static const std::string KINEMATIC_LIMITS_CHECK_TASK_NAME = "KINEMATIC_LIMITS_CHECK";

namespace snp_motion_planning
{
class KinematicLimitsCheckTaskInfo : public tesseract_planning::TaskInfo
{
public:
  using Ptr = std::shared_ptr<KinematicLimitsCheckTaskInfo>;
  using ConstPtr = std::shared_ptr<const KinematicLimitsCheckTaskInfo>;

  KinematicLimitsCheckTaskInfo(std::size_t unique_id, std::string name = KINEMATIC_LIMITS_CHECK_TASK_NAME)
    : TaskInfo(unique_id, std::move(name))
  {
  }
};

class KinematicLimitsCheckTaskGenerator : public tesseract_planning::TaskGenerator
{
public:
  KinematicLimitsCheckTaskGenerator(std::string name = KINEMATIC_LIMITS_CHECK_TASK_NAME)
    : tesseract_planning::TaskGenerator(std::move(name))
  {
  }

  int conditionalProcess(tesseract_planning::TaskInput input, std::size_t unique_id) const override
  {
    if (input.isAborted())
      return 0;

    auto info = std::make_shared<KinematicLimitsCheckTaskInfo>(unique_id, name_);
    info->return_value = 0;
    input.addTaskInfo(info);
    tesseract_common::Timer timer;
    timer.start();
    saveInputs(*info, input);

    // --------------------
    // Check that inputs are valid
    // --------------------
    tesseract_planning::Instruction* input_results = input.getResults();
    if (!isCompositeInstruction(*input_results))
    {
      saveOutputs(*info, input);
      info->elapsed_time = timer.elapsedSeconds();
      info->message = "Input results to kinematic limits check must be a composite instruction";
      return 0;
    }

    auto& ci = input_results->as<tesseract_planning::CompositeInstruction>();
    const tesseract_planning::ManipulatorInfo& manip_info = ci.getManipulatorInfo();

    // Get Composite Profile
    std::string profile_name = ci.getProfile();
    profile_name = tesseract_planning::getProfileString(name_, profile_name, input.composite_profile_remapping);
    auto profile = tesseract_planning::getProfile<KinematicLimitsCheckProfile>(
        name_, profile_name, *input.profiles, std::make_shared<KinematicLimitsCheckProfile>());
    profile = tesseract_planning::applyProfileOverrides(name_, profile_name, profile, ci.profile_overrides);

    // Create data structures for checking for plan profile overrides
    auto flattened = tesseract_planning::flatten(ci, tesseract_planning::moveFilter);
    if (flattened.empty())
    {
      info->return_value = 1;
      saveOutputs(*info, input);
      info->elapsed_time = timer.elapsedSeconds();
      info->message = "Kinematic limits check found no MoveInstructions to process";
      return 1;
    }

    // Wrap the composite instruction in a trajectory container
    tesseract_planning::TrajectoryContainer::Ptr trajectory =
        std::make_shared<tesseract_planning::InstructionsTrajectory>(ci);

    // Extract the motion group
    tesseract_kinematics::JointGroup::ConstPtr motion_group = input.env->getJointGroup(manip_info.manipulator);

    // Check the trajectory limits
    for (Eigen::Index i = 0; i < trajectory->size(); ++i)
    {
      const Eigen::VectorXd& joint_pos = trajectory->getPosition(i);
      const Eigen::VectorXd& joint_vel = trajectory->getVelocity(i);
      const Eigen::VectorXd& joint_acc = trajectory->getAcceleration(i);

      if (profile->check_position)
      {
        if (!tesseract_common::satisfiesPositionLimits(joint_pos, motion_group->getLimits().joint_limits))
        {
          std::stringstream ss;
          ss << "Joint position limit violation(s) at waypoint " << i;
          info->message = ss.str();
          return 0;
        }
      }

      if (profile->check_velocity)
      {
        // Check for joint velocity limit violations
        Eigen::Array<bool, Eigen::Dynamic, 1> vel_limit_violations =
            motion_group->getLimits().velocity_limits.array() < joint_vel.array().abs();
        if (vel_limit_violations.any())
        {
          Eigen::ArrayXd capacity = 100.0 * joint_vel.array().abs() / motion_group->getLimits().velocity_limits.array();
          std::stringstream ss;
          ss << "Joint velocity limit violation(s) at waypoint " << i << ": "
             << capacity.transpose().format(Eigen::IOFormat(4, 0, " ", "\n", "[", "]")) << " (%% capacity)";
          info->message = ss.str();

          return 0;
        }
      }

      if (profile->check_acceleration)
      {
        // Check for joint velocity acceleration limit violations
        Eigen::Array<bool, Eigen::Dynamic, 1> acc_limit_violations =
            motion_group->getLimits().acceleration_limits.array() < joint_acc.array().abs();
        if (acc_limit_violations.any())
        {
          Eigen::ArrayXd capacity =
              100.0 * joint_acc.array().abs() / motion_group->getLimits().acceleration_limits.array();
          std::stringstream ss;
          ss << "Joint acceleration limit violation(s) at waypoint " << i << ": "
             << capacity.transpose().format(Eigen::IOFormat(4, 0, " ", "\n", "[", "]")) << " (%% capacity)";
          info->message = ss.str();

          return 0;
        }
      }
    }

    info->return_value = 1;
    info->message = "Kinematic limits check succeeded";
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    return 1;
  }

  void process(tesseract_planning::TaskInput input, std::size_t unique_id) const override
  {
    conditionalProcess(input, unique_id);
  }
};

}  // namespace snp_motion_planning

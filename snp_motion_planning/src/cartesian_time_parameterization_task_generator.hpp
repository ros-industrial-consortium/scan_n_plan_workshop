#pragma once
#include "cartesian_time_parameterization.hpp"
#include "cartesian_time_parameterization_profile.hpp"

#include <tesseract_common/timer.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_process_managers/core/task_generator.h>
#include <tesseract_process_managers/core/utils.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils/filter_functions.h>
#include <tesseract_command_language/utils/flatten_utils.h>
#include <tesseract_time_parameterization/instructions_trajectory.h>

namespace snp_motion_planning
{
class CartesianTimeParameterizationTaskInfo : public tesseract_planning::TaskInfo
{
public:
  using Ptr = std::shared_ptr<CartesianTimeParameterizationTaskInfo>;
  using ConstPtr = std::shared_ptr<const CartesianTimeParameterizationTaskInfo>;

  CartesianTimeParameterizationTaskInfo(std::size_t unique_id, std::string name = "CARTESIAN_TIME_PARAMETERIZATION")
    : TaskInfo(unique_id, std::move(name))
  {
  }
};

class CartesianTimeParameterizationTaskGenerator : public tesseract_planning::TaskGenerator
{
public:
  CartesianTimeParameterizationTaskGenerator(std::string name = "CARTESIAN_TIME_PARAMETERIZATION")
    : tesseract_planning::TaskGenerator(std::move(name))
  {
  }

  int conditionalProcess(tesseract_planning::TaskInput input, std::size_t unique_id) const override
  {
    if (input.isAborted())
      return 0;

    auto info = std::make_shared<CartesianTimeParameterizationTaskInfo>(unique_id, name_);
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
      CONSOLE_BRIDGE_logError("Input results to iterative spline parameterization must be a composite instruction");
      saveOutputs(*info, input);
      info->elapsed_time = timer.elapsedSeconds();
      return 0;
    }

    auto& ci = input_results->as<tesseract_planning::CompositeInstruction>();
    const tesseract_planning::ManipulatorInfo& manip_info = ci.getManipulatorInfo();

    const double vel_trans = 0.250;
    const double vel_rot = M_PI / 2.0;
    const double acc_trans = vel_trans;
    const double acc_rot = vel_rot;
    auto default_profile =
        std::make_shared<CartesianTimeParameterizationProfile>(vel_trans, vel_rot, acc_trans, acc_rot);

    // Get Composite Profile
    std::string profile_name = ci.getProfile();
    profile_name = tesseract_planning::getProfileString(name_, profile_name, input.composite_profile_remapping);
    auto profile = tesseract_planning::getProfile<CartesianTimeParameterizationProfile>(
        name_, profile_name, *input.profiles, default_profile);
    profile = tesseract_planning::applyProfileOverrides(name_, profile_name, profile, ci.profile_overrides);

    // Create data structures for checking for plan profile overrides
    auto flattened = tesseract_planning::flatten(ci, tesseract_planning::moveFilter);
    if (flattened.empty())
    {
      CONSOLE_BRIDGE_logWarn("Cartesian time parameterization found no MoveInstructions to process");
      info->return_value = 1;
      saveOutputs(*info, input);
      info->elapsed_time = timer.elapsedSeconds();
      return 1;
    }

    // Solve using parameters
    tesseract_planning::TrajectoryContainer::Ptr trajectory =
        std::make_shared<tesseract_planning::InstructionsTrajectory>(ci);

    CartesianTimeParameterization solver(input.env, manip_info.manipulator, manip_info.tcp_frame,
                                         profile->max_translational_velocity, profile->max_rotational_velocity,
                                         profile->max_translational_acceleration, profile->max_rotational_acceleration);

    if (!solver.compute(*trajectory, profile->max_velocity_scaling_factor, profile->max_acceleration_scaling_factor))
    {
      CONSOLE_BRIDGE_logInform("Failed to perform iterative spline time parameterization for process input: %s!",
                               input_results->getDescription().c_str());
      saveOutputs(*info, input);
      info->elapsed_time = timer.elapsedSeconds();
      return 0;
    }

    CONSOLE_BRIDGE_logDebug("Iterative spline time parameterization succeeded");
    info->return_value = 1;
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

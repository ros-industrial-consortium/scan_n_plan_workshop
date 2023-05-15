#pragma once
#include "cartesian_time_parameterization.hpp"

#include <tesseract_common/timer.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_process_managers/core/task_generator.h>
#include <tesseract_process_managers/core/utils.h>
#include <tesseract_process_managers/task_profiles/interative_spline_parameterization_profile.h>
#include <tesseract_process_managers/task_generators/iterative_spline_parameterization_task_generator.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils/filter_functions.h>
#include <tesseract_command_language/utils/flatten_utils.h>
#include <tesseract_time_parameterization/instructions_trajectory.h>

namespace snp_motion_planning
{
class CartesianTimeParameterizationTaskGenerator : public tesseract_planning::TaskGenerator
{
public:
  CartesianTimeParameterizationTaskGenerator(std::string name = "Cartesian Time Parameterization")
    : tesseract_planning::TaskGenerator(std::move(name))
  {
  }

  int conditionalProcess(tesseract_planning::TaskInput input, std::size_t unique_id) const override
  {
    if (input.isAborted())
      return 0;

    auto info = std::make_shared<tesseract_planning::IterativeSplineParameterizationTaskInfo>(unique_id, name_);
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
    CartesianTimeParameterization solver(input.env, manip_info.manipulator, manip_info.tcp_frame, 0.25, 0.25);

    // Get Composite Profile
    std::string profile = ci.getProfile();
    profile = tesseract_planning::getProfileString(name_, profile, input.composite_profile_remapping);
    auto cur_composite_profile = tesseract_planning::getProfile<tesseract_planning::IterativeSplineParameterizationProfile>(
        name_, profile, *input.profiles, std::make_shared<tesseract_planning::IterativeSplineParameterizationProfile>());
    cur_composite_profile = tesseract_planning::applyProfileOverrides(name_, profile, cur_composite_profile, ci.profile_overrides);

    // Create data structures for checking for plan profile overrides
    auto flattened = tesseract_planning::flatten(ci, tesseract_planning::moveFilter);
    if (flattened.empty())
    {
      CONSOLE_BRIDGE_logWarn("Iterative spline time parameterization found no MoveInstructions to process");
      info->return_value = 1;
      saveOutputs(*info, input);
      info->elapsed_time = timer.elapsedSeconds();
      return 1;
    }

    Eigen::VectorXd velocity_scaling_factors = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(flattened.size())) *
                                               cur_composite_profile->max_velocity_scaling_factor;
    Eigen::VectorXd acceleration_scaling_factors = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(flattened.size())) *
                                                   cur_composite_profile->max_acceleration_scaling_factor;

    // Loop over all PlanInstructions
    for (Eigen::Index idx = 0; idx < static_cast<Eigen::Index>(flattened.size()); idx++)
    {
      const auto& mi = flattened[static_cast<std::size_t>(idx)].get().as<tesseract_planning::MoveInstruction>();
      std::string plan_profile = mi.getProfile();

      // Check for remapping of the plan profile
      plan_profile = tesseract_planning::getProfileString(name_, profile, input.plan_profile_remapping);
      auto cur_move_profile = tesseract_planning::getProfile<tesseract_planning::IterativeSplineParameterizationProfile>(
          name_, plan_profile, *input.profiles, std::make_shared<tesseract_planning::IterativeSplineParameterizationProfile>());
      cur_move_profile = tesseract_planning::applyProfileOverrides(name_, profile, cur_move_profile, mi.profile_overrides);

      // If there is a move profile associated with it, override the parameters
      if (cur_move_profile)
      {
        velocity_scaling_factors[idx] = cur_move_profile->max_velocity_scaling_factor;
        acceleration_scaling_factors[idx] = cur_move_profile->max_acceleration_scaling_factor;
      }
    }

    // Solve using parameters
    tesseract_planning::TrajectoryContainer::Ptr trajectory = std::make_shared<tesseract_planning::InstructionsTrajectory>(ci);
    if (!solver.compute(*trajectory))
//, limits.velocity_limits, limits.acceleration_limits, velocity_scaling_factors,
//                         acceleration_scaling_factors))
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

}  // namespace tesseract_planning

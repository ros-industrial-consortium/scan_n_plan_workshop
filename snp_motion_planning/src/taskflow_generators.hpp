#pragma once

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
// Task Generators
#include <tesseract_process_managers/task_generators/check_input_task_generator.h>
#include <tesseract_process_managers/task_generators/has_seed_task_generator.h>
#include <tesseract_process_managers/task_generators/seed_min_length_task_generator.h>
#include <tesseract_process_managers/task_generators/motion_planner_task_generator.h>
#include <tesseract_process_managers/task_generators/discrete_contact_check_task_generator.h>
#include <tesseract_process_managers/task_generators/iterative_spline_parameterization_task_generator.h>
// Taskflow generators
#include <tesseract_process_managers/taskflow_generators/graph_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_taskflow.h>
#include <tesseract_process_managers/core/default_process_planners.h>

/**
 * @brief Creates a task flow graph for planning transition moves using a simple planner and TrajOpt with time
 * parameterization
 * @return
 */
tesseract_planning::TaskflowGenerator::UPtr createTransitionTaskflow()
{
  using namespace tesseract_planning;

  // Create the graph task flow
  auto graph = std::make_unique<GraphTaskflow>();

  // Input/seed checks
  auto check_input = graph->addNode(std::make_unique<CheckInputTaskGenerator>(), true);
  int has_seed = graph->addNode(std::make_unique<HasSeedTaskGenerator>(), true);
  int seed_min_length = graph->addNode(std::make_unique<SeedMinLengthTaskGenerator>(), true);

  // Simple planner with post-collision check
  auto simple_planner = std::make_shared<SimpleMotionPlanner>();
  int simple = graph->addNode(std::make_unique<MotionPlannerTaskGenerator>(simple_planner), true);
  int simple_collision = graph->addNode(std::make_unique<DiscreteContactCheckTaskGenerator>(), true);

  // TrajOpt with post-collision check
  auto trajopt_planner = std::make_shared<TrajOptMotionPlanner>();
  int trajopt = graph->addNode(std::make_unique<MotionPlannerTaskGenerator>(trajopt_planner), true);
  int trajopt_collision = graph->addNode(std::make_unique<DiscreteContactCheckTaskGenerator>(), true);

  // Time parameterization
  int time_param = graph->addNode(std::make_unique<IterativeSplineParameterizationTaskGenerator>(), true);

  graph->addEdges(check_input, { GraphTaskflow::ERROR_NODE, has_seed });
  graph->addEdges(has_seed, { simple, seed_min_length });
  graph->addEdges(seed_min_length, { GraphTaskflow::ERROR_NODE, simple_collision });
  graph->addEdges(simple, { GraphTaskflow::ERROR_NODE, simple_collision });
  graph->addEdges(simple_collision, { trajopt, time_param });
  graph->addEdges(trajopt, { GraphTaskflow::ERROR_NODE, trajopt_collision });
  graph->addEdges(trajopt_collision, { GraphTaskflow::ERROR_NODE, time_param });
  graph->addEdges(time_param, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });

  return graph;
}

/**
 * @brief Creates a task flow graph for planning freespace motions using a simple planner, TrajOpt, and OMPL smoothed by
 * TrajOpt, with time parameterization
 * @return
 */
tesseract_planning::TaskflowGenerator::UPtr createFreespaceTaskflow()
{
  using namespace tesseract_planning;

  // Create the graph task flow
  auto graph = std::make_unique<GraphTaskflow>();

  // Input/seed checks
  auto check_input = graph->addNode(std::make_unique<CheckInputTaskGenerator>(), true);
  int has_seed = graph->addNode(std::make_unique<HasSeedTaskGenerator>(), true);
  int seed_min_length = graph->addNode(std::make_unique<SeedMinLengthTaskGenerator>(), true);

  // Simple planner with post-collision check
  auto simple_planner = std::make_shared<SimpleMotionPlanner>();
  int simple = graph->addNode(std::make_unique<MotionPlannerTaskGenerator>(simple_planner), true);
  int simple_collision = graph->addNode(std::make_unique<DiscreteContactCheckTaskGenerator>(), true);

  // TrajOpt with post-collision check
  auto trajopt_planner = std::make_shared<TrajOptMotionPlanner>();
  int trajopt = graph->addNode(std::make_unique<MotionPlannerTaskGenerator>(trajopt_planner), true);
  int trajopt_collision = graph->addNode(std::make_unique<DiscreteContactCheckTaskGenerator>(), true);

  // OMPL smoothed by TrajOpt with post-plan collision check
  auto ompl_planner = std::make_shared<OMPLMotionPlanner>();
  int ompl = graph->addNode(std::make_unique<MotionPlannerTaskGenerator>(ompl_planner), true);
  auto ompl_trajopt_planner = std::make_shared<TrajOptMotionPlanner>();
  int ompl_trajopt = graph->addNode(std::make_unique<MotionPlannerTaskGenerator>(ompl_trajopt_planner), true);
  int ompl_collision = graph->addNode(std::make_unique<DiscreteContactCheckTaskGenerator>(), true);

  // Time parameterization
  int time_param = graph->addNode(std::make_unique<IterativeSplineParameterizationTaskGenerator>(), true);

  graph->addEdges(check_input, { GraphTaskflow::ERROR_NODE, has_seed });
  graph->addEdges(has_seed, { simple, seed_min_length });
  graph->addEdges(seed_min_length, { GraphTaskflow::ERROR_NODE, simple_collision });
  graph->addEdges(simple, { GraphTaskflow::ERROR_NODE, simple_collision });
  graph->addEdges(simple_collision, { trajopt, time_param });
  graph->addEdges(trajopt, { ompl, trajopt_collision });
  graph->addEdges(trajopt_collision, { ompl, time_param });
  graph->addEdges(ompl, { GraphTaskflow::ERROR_NODE, ompl_trajopt });
  graph->addEdges(ompl_trajopt, { GraphTaskflow::ERROR_NODE, ompl_collision });
  graph->addEdges(ompl_collision, { GraphTaskflow::ERROR_NODE, time_param });
  graph->addEdges(time_param, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });

  return graph;
}

/**
 * @brief Creates a raster taskflow using the custom-defined freespace and transition planning taskflows
 */
tesseract_planning::TaskflowGenerator::UPtr createRasterTaskflow()
{
  using namespace tesseract_planning;
  return std::make_unique<RasterTaskflow>(createFreespaceTaskflow(), createTransitionTaskflow(),
                                          createCartesianGenerator());
}

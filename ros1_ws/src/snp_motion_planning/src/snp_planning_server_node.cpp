/**
 * @file twc_planning_server_node.cpp
 * @brief The Tesseract Workcell planning server node
 *
 * @author Levi Armstrong
 * @date August 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_planning_server/tesseract_planning_server.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <descartes_light/edge_evaluators/compound_edge_evaluator.h>

#include <tesseract_process_managers/taskflow_generators/descartes_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/trajopt_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/freespace_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_global_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_taskflow.h>

#include <snp_motion_planning/utils.h>

using tesseract_planning_server::TesseractPlanningServer;
using tesseract_planning::ProfileDictionary;
using tesseract_planning::OMPLPlanProfile;
using tesseract_planning::OMPLDefaultPlanProfile;
using tesseract_planning::TrajOptCompositeProfile;
using tesseract_planning::TrajOptPlanProfile;
using tesseract_planning::TrajOptDefaultCompositeProfile;
using tesseract_planning::TrajOptDefaultPlanProfile;
using tesseract_planning::DescartesPlanProfile;
using tesseract_planning::DescartesDefaultPlanProfileD;
using tesseract_planning::TaskflowGenerator;
using tesseract_planning::DescartesTaskflow;
using tesseract_planning::DescartesTaskflowParams;
using tesseract_planning::TrajOptTaskflow;
using tesseract_planning::TrajOptTaskflowParams;
using tesseract_planning::FreespaceTaskflow;
using tesseract_planning::FreespaceTaskflowParams;
using tesseract_planning::RasterGlobalTaskflow;
using tesseract_planning::RasterTaskflow;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const double LONGEST_VALID_SEGMENT_LENGTH = 0.01;
const double CONTACT_DISTANCE_THRESHOLD = 0.01;

std::shared_ptr<tesseract_planning::TrajOptDefaultCompositeProfile>
createTrajOptCompositeProfile()
{
  auto trajopt_composite_profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
  trajopt_composite_profile->longest_valid_segment_length = LONGEST_VALID_SEGMENT_LENGTH;
  trajopt_composite_profile->collision_cost_config.enabled = true;
  trajopt_composite_profile->collision_cost_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  trajopt_composite_profile->collision_cost_config.safety_margin = 2 * CONTACT_DISTANCE_THRESHOLD;
  trajopt_composite_profile->collision_cost_config.coeff = 10;
  trajopt_composite_profile->collision_constraint_config.enabled = true;
  trajopt_composite_profile->collision_constraint_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  trajopt_composite_profile->collision_constraint_config.safety_margin = CONTACT_DISTANCE_THRESHOLD;
  trajopt_composite_profile->collision_constraint_config.safety_margin_buffer = 2 * CONTACT_DISTANCE_THRESHOLD;
  trajopt_composite_profile->collision_constraint_config.coeff = 1;
  trajopt_composite_profile->smooth_velocities = false;
  trajopt_composite_profile->smooth_accelerations = true;
  trajopt_composite_profile->smooth_jerks = true;

  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(6);
  trajopt_composite_profile->velocity_coeff = 5 * joint_weights;
  trajopt_composite_profile->acceleration_coeff = 10 * joint_weights;
  trajopt_composite_profile->jerk_coeff = 15 * joint_weights;

  return trajopt_composite_profile;
}

std::shared_ptr<tesseract_planning::TrajOptDefaultPlanProfile> createTrajOptPlanProfile()
{
  auto trajopt_plan_profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();

  // Tool z-axis free
  trajopt_plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 10);
  trajopt_plan_profile->cartesian_coeff(5) = 0;
  trajopt_plan_profile->term_type = trajopt::TermType::TT_COST;

  return trajopt_plan_profile;
}

std::shared_ptr<tesseract_planning::OMPLDefaultPlanProfile> createOMPLPlanProfile()
{
  auto ompl_plan_profile = std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>();
  ompl_plan_profile->collision_check_config.collision_margin_override_type =
      tesseract_collision::CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN;
  ompl_plan_profile->collision_check_config.collision_margin_data =
      tesseract_collision::CollisionMarginData(CONTACT_DISTANCE_THRESHOLD);
  ompl_plan_profile->collision_check_config.longest_valid_segment_length = LONGEST_VALID_SEGMENT_LENGTH;
  ompl_plan_profile->collision_check_config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
  ompl_plan_profile->planning_time = 120;

  auto rrtc1 = std::make_shared<tesseract_planning::RRTConnectConfigurator>();
  rrtc1->range = 0.1;

  auto rrtc2 = std::make_shared<tesseract_planning::RRTConnectConfigurator>();
  rrtc2->range = 0.2;

  auto est1 = std::make_shared<tesseract_planning::ESTConfigurator>();
  est1->range = 0.1;
  est1->goal_bias = 0.5;

  auto est2 = std::make_shared<tesseract_planning::ESTConfigurator>();
  est2->range = 0.1;
  est2->goal_bias = 0.25;

  auto est3 = std::make_shared<tesseract_planning::ESTConfigurator>();
  est3->range = 0.1;
  est3->goal_bias = 0.75;

  ompl_plan_profile->planners.clear();
  ompl_plan_profile->planners.push_back(rrtc1);
  ompl_plan_profile->planners.push_back(rrtc1);
  ompl_plan_profile->planners.push_back(rrtc1);
  ompl_plan_profile->planners.push_back(rrtc2);
  ompl_plan_profile->planners.push_back(rrtc2);
  ompl_plan_profile->planners.push_back(rrtc2);
  ompl_plan_profile->planners.push_back(est1);
  ompl_plan_profile->planners.push_back(est2);
  ompl_plan_profile->planners.push_back(est3);
  ompl_plan_profile->planners.push_back(est1);
  ompl_plan_profile->planners.push_back(est2);
  ompl_plan_profile->planners.push_back(est3);

  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(6);

  ompl_plan_profile->state_sampler_allocator = [joint_weights](const ompl::base::StateSpace* space,
                                                  const tesseract_planning::OMPLProblem& prob) {
    const auto& limits = prob.manip->getLimits().joint_limits;
    return tesseract_planning::allocWeightedRealVectorStateSampler(space, joint_weights, limits);
  };

  return ompl_plan_profile;
}

std::shared_ptr<tesseract_planning::DescartesDefaultPlanProfile<float>>
createDescartesPlanProfile()
{
  auto descartes_plan_profile = std::make_shared<tesseract_planning::DescartesDefaultPlanProfile<float>>();
  descartes_plan_profile->allow_collision = true;
  descartes_plan_profile->vertex_collision_check_config.collision_margin_override_type =
      tesseract_collision::CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN;
  descartes_plan_profile->vertex_collision_check_config.collision_margin_data =
      tesseract_collision::CollisionMarginData(1.5 * CONTACT_DISTANCE_THRESHOLD);
  descartes_plan_profile->num_threads = static_cast<int>(std::thread::hardware_concurrency());

  descartes_plan_profile->target_pose_sampler = [](const Eigen::Isometry3d& pose){return tesseract_planning::sampleToolZAxis(pose, M_PI_4); };

  // Add Vertex Evaluator
  descartes_plan_profile->vertex_evaluator = [](const tesseract_planning::DescartesProblem<float>& prob) {
    return std::make_shared<snp::DescartesStateValidator>(prob.manip, "robot_base_link", "robot_tool0");
  };

  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(6);

  descartes_plan_profile->edge_evaluator = [joint_weights](const tesseract_planning::DescartesProblem<float>& prob) {
        auto e = std::make_shared<descartes_light::CompoundEdgeEvaluator<float>>();
        e->evaluators.push_back(std::make_shared<snp::RobotConfigEdgeEvaluator<float>>(prob.manip, "robot_base_link", "robot_tool0"));
        e->evaluators.push_back(std::make_shared<snp::WeightedEuclideanDistanceEdgeEvaluator<float>>(joint_weights));
        return e;
      };

  return descartes_plan_profile;
}

void loadTWCDefaultProfiles(TesseractPlanningServer& planning_server)
{
  ProfileDictionary::Ptr dict = planning_server.getProcessPlanningServer().getProfiles();

  { // Trajopt Composite Profiles
    auto p = createTrajOptCompositeProfile();
    dict->addProfile<TrajOptCompositeProfile>("FREESPACE", p);
    dict->addProfile<TrajOptCompositeProfile>("TRANSITION", p);
    dict->addProfile<TrajOptCompositeProfile>("RASTER", p);
  }

  { // Trajopt Plan Profiles
    auto p = createTrajOptPlanProfile();
    dict->addProfile<TrajOptPlanProfile>("FREESPACE", p);
    dict->addProfile<TrajOptPlanProfile>("TRANSITION", p);
    dict->addProfile<TrajOptPlanProfile>("RASTER", p);
  }

  { // OMPL Plan Profiles
    auto p = createOMPLPlanProfile();
    dict->addProfile<OMPLPlanProfile>("FREESPACE", p);
    dict->addProfile<OMPLPlanProfile>("TRANSITION", p);
    dict->addProfile<OMPLPlanProfile>("RASTER", p);
  }

  { // Descartes Plan Profiles
    auto p = createDescartesPlanProfile();
    dict->addProfile<DescartesPlanProfile<float>>("FREESPACE", p);
    dict->addProfile<DescartesPlanProfile<float>>("TRANSITION", p);
    dict->addProfile<DescartesPlanProfile<float>>("RASTER", p);
  }
}

TaskflowGenerator::UPtr createRasterDebugGenerator()
{
  tesseract_planning::DescartesTaskflowParams params;
  params.enable_post_contact_discrete_check = false;
  params.enable_post_contact_continuous_check = false;
  params.enable_time_parameterization = true;

  TaskflowGenerator::UPtr freespace_task = std::make_unique<DescartesTaskflow>(params);
  TaskflowGenerator::UPtr transition_task = std::make_unique<DescartesTaskflow>(params);
  TaskflowGenerator::UPtr raster_task = std::make_unique<DescartesTaskflow>(params);

  return std::make_unique<RasterTaskflow>(std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterGlobalDebugGenerator()
{
  tesseract_planning::DescartesTaskflowParams params;
  params.enable_post_contact_discrete_check = false;
  params.enable_post_contact_continuous_check = false;
  params.enable_time_parameterization = true;

  return std::make_unique<tesseract_planning::DescartesTaskflow>(params);
}

TaskflowGenerator::UPtr createRasterGlobalNoPostCheckGenerator()
{
  tesseract_planning::DescartesTaskflowParams global_params;
  global_params.enable_post_contact_discrete_check = false;
  global_params.enable_post_contact_continuous_check = false;
  global_params.enable_time_parameterization = false;
  TaskflowGenerator::UPtr global_task = std::make_unique<DescartesTaskflow>(global_params);

  tesseract_planning::FreespaceTaskflowParams freespace_params;
  freespace_params.enable_post_contact_discrete_check = false;
  freespace_params.enable_post_contact_continuous_check = false;
  freespace_params.type = tesseract_planning::FreespaceTaskflowType::TRAJOPT_FIRST;
  TaskflowGenerator::UPtr freespace_task = std::make_unique<tesseract_planning::FreespaceTaskflow>(freespace_params);

  tesseract_planning::FreespaceTaskflowParams transition_params;
  transition_params.enable_post_contact_discrete_check = false;
  transition_params.enable_post_contact_continuous_check = false;
  transition_params.type = tesseract_planning::FreespaceTaskflowType::TRAJOPT_FIRST;
  TaskflowGenerator::UPtr transition_task = std::make_unique<tesseract_planning::FreespaceTaskflow>(transition_params);

  tesseract_planning::TrajOptTaskflowParams raster_params;
  raster_params.enable_post_contact_discrete_check = false;
  raster_params.enable_post_contact_continuous_check = false;
  TaskflowGenerator::UPtr raster_task = std::make_unique<tesseract_planning::TrajOptTaskflow>(raster_params);

  return std::make_unique<tesseract_planning::RasterGlobalTaskflow>(
      std::move(global_task), std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterTrajOptGenerator()
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams fparams;
  TaskflowGenerator::UPtr freespace_task = std::make_unique<FreespaceTaskflow>(fparams);
  TaskflowGenerator::UPtr transition_task = std::make_unique<FreespaceTaskflow>(fparams);

  // Create Raster Taskflow
  tesseract_planning::TrajOptTaskflowParams raster_params;
  TaskflowGenerator::UPtr raster_task = std::make_unique<tesseract_planning::TrajOptTaskflow>(raster_params);

  return std::make_unique<RasterTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "snp_planning_server");
  ros::NodeHandle pnh("~");
  std::string robot_description;
  std::string discrete_plugin;
  std::string continuous_plugin;
  std::string monitor_namespace;
  std::string monitored_namespace;
  bool publish_environment{ false };
  int threads = static_cast<int>(std::thread::hardware_concurrency());

//   console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  if (!pnh.getParam("monitor_namespace", monitor_namespace))
  {
    ROS_ERROR("Missing required parameter monitor_namespace!");
    return 1;
  }

  pnh.param<std::string>("monitored_namespace", monitored_namespace, "");
  pnh.param<std::string>("robot_description", robot_description, ROBOT_DESCRIPTION_PARAM);
  pnh.param<std::string>("discrete_plugin", discrete_plugin, "");
  pnh.param<std::string>("continuous_plugin", continuous_plugin, "");
  pnh.param<bool>("publish_environment", publish_environment, publish_environment);
  pnh.param<int>("threads", threads, threads);

  TesseractPlanningServer planning_server(robot_description, monitor_namespace, static_cast<std::size_t>(threads), discrete_plugin, continuous_plugin);
  loadTWCDefaultProfiles(planning_server);

  planning_server.getProcessPlanningServer().registerProcessPlanner("RasterTrajOpt", createRasterTrajOptGenerator());
  planning_server.getProcessPlanningServer().registerProcessPlanner("RasterDebug", createRasterDebugGenerator());
  planning_server.getProcessPlanningServer().registerProcessPlanner("RasterGDebug", createRasterGlobalDebugGenerator());
  planning_server.getProcessPlanningServer().registerProcessPlanner("RasterGNoPostCheckDebug", createRasterGlobalNoPostCheckGenerator());

  if (publish_environment)
    planning_server.getEnvironmentMonitor().startPublishingEnvironment();

  if (!monitored_namespace.empty())
    planning_server.getEnvironmentMonitor().startMonitoringEnvironment(monitored_namespace);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::waitForShutdown();

  return 0;
}

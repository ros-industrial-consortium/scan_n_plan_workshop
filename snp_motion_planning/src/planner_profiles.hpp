#pragma once

#include <thread>
#include <descartes_light/edge_evaluators/compound_edge_evaluator.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_ladder_graph_solver_profile.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_real_vector_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>
#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>

static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";
static const std::string OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask";
static const std::string DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask";
static const std::string SIMPLE_DEFAULT_NAMESPACE = "SimpleMotionPlannerTask";
static const std::string MIN_LENGTH_DEFAULT_NAMESPACE = "MinLengthTask";
static const std::string CONTACT_CHECK_DEFAULT_NAMESPACE = "DiscreteContactCheckTask";
static const std::string ISP_DEFAULT_NAMESPACE = "IterativeSplineParameterizationTask";

/**
 * @brief Container for a collision pair with a specifiable minimum allowable contact distance between the link pair
 */
struct ExplicitCollisionPair
{
  explicit ExplicitCollisionPair(std::string first_, std::string second_, double distance_)
    : first(first_), second(second_), distance(distance_)
  {
  }

  /** @brief Name of the first link */
  std::string first;
  /** @brief Name of the second link */
  std::string second;
  /** @brief Minimum allowable contact distance between the two links */
  double distance;
};

template <typename FloatType>
typename tesseract_planning::DescartesLadderGraphSolverProfile<FloatType>::Ptr createDescartesSolverProfile()
{
  auto profile = std::make_shared<tesseract_planning::DescartesLadderGraphSolverProfile<FloatType>>();
  profile->num_threads = static_cast<int>(std::thread::hardware_concurrency());
  return profile;
}

template <typename FloatType>
typename tesseract_planning::DescartesDefaultPlanProfile<FloatType>::Ptr
createDescartesPlanProfile(FloatType min_contact_distance,
                           const std::vector<ExplicitCollisionPair>& unique_collision_pairs,
                           const FloatType longest_valid_segment_length)
{
  auto profile = std::make_shared<tesseract_planning::DescartesDefaultPlanProfile<FloatType>>();
  profile->use_redundant_joint_solutions = false;

  // Tool pose sampler
  profile->target_pose_fixed = false;
  profile->target_pose_sample_axis = Eigen::Vector3d::UnitZ();
  profile->target_pose_sample_resolution = 10.0 * M_PI / 180.0;
  profile->target_pose_sample_min = -M_PI;
  profile->target_pose_sample_max = M_PI - profile->target_pose_sample_resolution;

  // Collision checking
  profile->allow_collision = false;
  profile->enable_collision = true;
  profile->vertex_collision_check_config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  profile->vertex_collision_check_config.contact_manager_config.margin_data.setDefaultCollisionMargin(
      min_contact_distance);
  profile->vertex_collision_check_config.longest_valid_segment_length = longest_valid_segment_length;

  profile->vertex_collision_check_config.contact_manager_config.margin_data_override_type =
      tesseract_common::CollisionMarginOverrideType::MODIFY;
  for (const ExplicitCollisionPair& pair : unique_collision_pairs)
    profile->vertex_collision_check_config.contact_manager_config.margin_data.setPairCollisionMargin(
        pair.first, pair.second, pair.distance);

  profile->enable_edge_collision = false;
  profile->edge_collision_check_config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  profile->edge_collision_check_config.contact_manager_config.margin_data.setDefaultCollisionMargin(
      min_contact_distance);
  profile->edge_collision_check_config.longest_valid_segment_length = longest_valid_segment_length;

  profile->edge_collision_check_config.contact_manager_config.margin_data_override_type =
      tesseract_common::CollisionMarginOverrideType::MODIFY;
  for (const ExplicitCollisionPair& pair : unique_collision_pairs)
    profile->edge_collision_check_config.contact_manager_config.margin_data.setPairCollisionMargin(
        pair.first, pair.second, pair.distance);

  return profile;
}

tesseract_planning::OMPLRealVectorPlanProfile::Ptr
createOMPLProfile(const double min_contact_distance, const std::vector<ExplicitCollisionPair>& unique_collision_pairs,
                  const double longest_valid_segment_length)
{
  // OMPL freespace and transition profiles
  // Create the RRT parameters
  auto n = static_cast<Eigen::Index>(std::thread::hardware_concurrency());
  auto range = Eigen::VectorXd::LinSpaced(n, 0.05, 0.5);

  // Add as many planners as available threads so mulitple OMPL plans can happen in parallel
  auto profile = std::make_shared<tesseract_planning::OMPLRealVectorPlanProfile>();
  profile->solver_config.planning_time = 5.0;
  profile->solver_config.max_solutions = 1;

  profile->solver_config.planners.clear();
  profile->solver_config.planners.reserve(static_cast<std::size_t>(n));
  for (Eigen::Index i = 0; i < n; ++i)
  {
    auto rrt = std::make_shared<tesseract_planning::RRTConnectConfigurator>();
    rrt->range = range(i);
    profile->solver_config.planners.push_back(rrt);
  }

  // Collision checking
  profile->collision_check_config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  profile->collision_check_config.contact_manager_config.margin_data.setDefaultCollisionMargin(min_contact_distance);
  profile->collision_check_config.contact_manager_config.margin_data_override_type =
      tesseract_common::CollisionMarginOverrideType::MODIFY;
  profile->collision_check_config.longest_valid_segment_length = longest_valid_segment_length;

  for (const ExplicitCollisionPair& pair : unique_collision_pairs)
    profile->collision_check_config.contact_manager_config.margin_data.setPairCollisionMargin(pair.first, pair.second,
                                                                                              pair.distance);

  return profile;
}

std::shared_ptr<tesseract_planning::TrajOptPlanProfile>
createTrajOptToolZFreePlanProfile(const Eigen::VectorXd& cart_tolerance = Eigen::VectorXd::Zero(6),
                                  const Eigen::VectorXd& cart_coeff = Eigen::VectorXd::Constant(6, 1, 2.5))
{
  auto profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();

  profile->cartesian_cost_config.enabled = true;
  profile->cartesian_cost_config.use_tolerance_override = true;
  profile->cartesian_cost_config.lower_tolerance = Eigen::VectorXd::Zero(6);
  profile->cartesian_cost_config.upper_tolerance = Eigen::VectorXd::Zero(6);
  profile->cartesian_cost_config.coeff = cart_coeff;

  profile->cartesian_constraint_config.enabled = true;
  profile->cartesian_constraint_config.use_tolerance_override = true;
  profile->cartesian_constraint_config.lower_tolerance = -1 * cart_tolerance;
  profile->cartesian_constraint_config.upper_tolerance = cart_tolerance;
  profile->cartesian_constraint_config.coeff = cart_coeff;

  profile->joint_cost_config.enabled = false;
  profile->joint_cost_config.coeff = Eigen::VectorXd::Constant(1, 1, 5);

  profile->joint_constraint_config.enabled = true;
  profile->joint_constraint_config.coeff = Eigen::VectorXd::Constant(1, 1, 5);

  return profile;
}

std::shared_ptr<tesseract_planning::TrajOptDefaultCompositeProfile>
createTrajOptProfile(double min_contact_distance, const std::vector<ExplicitCollisionPair>& unique_collision_pairs,
                     double longest_valid_segment_length)
{
  // TrajOpt profiles
  auto profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
  profile->smooth_velocities = false;
  profile->velocity_coeff = Eigen::VectorXd::Constant(1, 1, 10.0);
  profile->smooth_accelerations = true;
  profile->acceleration_coeff = Eigen::VectorXd::Constant(1, 1, 25.0);
  profile->smooth_jerks = true;
  profile->jerk_coeff = Eigen::VectorXd::Constant(1, 1, 50.0);

  profile->contact_test_type = tesseract_collision::ContactTestType::CLOSEST;

  /* Safety margin notes:
   *   - A non-zero cost for distance to collision is computed only when the distance to collision < safety margin
   *   - The gradient of the collision cost is computed when the distance to collision is < safety margin + safety
   * margin buffer
   *   - When safety margin + safety margin buffer > distance to collision > safety margin, the gradient of the cost is
   * computed to prevent optimization instability, but the computed collision cost will be zero
   *
   * Therefore, we should set the safety margin relatively high such that non-zero collision costs are computed and
   * drive the robot away from collision. We can make the safety margin buffer a fixed size (usually 2cm is appropriate)
   * as the safety margin to ensure that the optimization does not move in the direction of collision even when the cost
   * is zero.
   */

  // Collision cost
  profile->collision_cost_config.enabled = true;
  if (profile->collision_cost_config.enabled)
  {
    profile->collision_cost_config.coeff = 10.0;
    profile->collision_cost_config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    profile->longest_valid_segment_length = longest_valid_segment_length;

    // Use a relatively large safety margin to drive the robot away from collision
    profile->collision_cost_config.safety_margin = std::max(0.025, min_contact_distance);
    // Use a small, fixed-size buffer beyond the defined margin to continue calculating collision avoidance gradients
    // even when the collision avoidance cost is zero
    profile->collision_cost_config.safety_margin_buffer = 0.025;

    if (!unique_collision_pairs.empty())
    {
      profile->special_collision_cost = std::make_shared<trajopt_common::SafetyMarginData>(
          profile->collision_cost_config.safety_margin, profile->collision_cost_config.coeff);

      // Populate the special collision cost
      for (const ExplicitCollisionPair& pair : unique_collision_pairs)
        profile->special_collision_cost->setPairSafetyMarginData(pair.first, pair.second, pair.distance,
                                                                 profile->collision_cost_config.coeff);
    }
  }

  // Collision constraint
  profile->collision_constraint_config.enabled = true;
  if (profile->collision_constraint_config.enabled)
  {
    profile->collision_constraint_config.coeff = 10.0;
    profile->collision_constraint_config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;

    // Use a minimal safety margin, such that the constraint is only violated (and therefore a cost produced) when the
    // robot is actually considered to be in collision
    profile->collision_constraint_config.safety_margin = min_contact_distance;
    // Use a small, fixed-size buffer beyond the defined margin to continue calculating collision avoidance gradients
    // even when the collision avoidance cost is zero
    profile->collision_constraint_config.safety_margin_buffer = 0.025;

    if (!unique_collision_pairs.empty())
    {
      profile->special_collision_constraint = std::make_shared<trajopt_common::SafetyMarginData>(
          min_contact_distance, profile->collision_constraint_config.coeff);

      // Populate the special collision constraint
      for (const ExplicitCollisionPair& pair : unique_collision_pairs)
        profile->special_collision_constraint->setPairSafetyMarginData(pair.first, pair.second, pair.distance,
                                                                       profile->collision_constraint_config.coeff);
    }
  }

  return profile;
}

std::shared_ptr<tesseract_planning::SimplePlannerLVSPlanProfile> createSimplePlannerProfile()
{
  return std::make_shared<tesseract_planning::SimplePlannerLVSPlanProfile>(5 * M_PI / 180, 0.1, 5 * M_PI / 180, 1);
}

tesseract_planning::ContactCheckProfile::Ptr
createContactCheckProfile(double longest_valid_segment_distance, double min_contact_distance,
                          const std::vector<ExplicitCollisionPair>& collision_pairs)
{
  auto profile =
      std::make_shared<tesseract_planning::ContactCheckProfile>(longest_valid_segment_distance, min_contact_distance);
  profile->config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  profile->config.contact_manager_config.margin_data_override_type =
      tesseract_common::CollisionMarginOverrideType::MODIFY;
  for (const ExplicitCollisionPair& pair : collision_pairs)
    profile->config.contact_manager_config.margin_data.setPairCollisionMargin(pair.first, pair.second, pair.distance);

  return profile;
}

#include "planner_profiles.hpp"
#include "taskflow_generators.hpp"
#include <tesseract_process_managers/core/process_planning_server.h>
#include <tesseract_motion_planners/default_planner_namespaces.h>
#include <tesseract_process_managers/task_profiles/check_input_profile.h>
#include <tesseract_process_managers/task_profiles/seed_min_length_profile.h>
#include <tesseract_process_managers/task_profiles/interative_spline_parameterization_profile.h>

#include <rclcpp/rclcpp.hpp>
#include <snp_msgs/srv/generate_motion_plan.hpp>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_rosutils/utils.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace tesseract_planning;

static const std::string TRANSITION_PLANNER = "TRANSITION";
static const std::string FREESPACE_PLANNER = "FREESPACE";
static const std::string RASTER_PLANNER = "RASTER";
static const std::string PROFILE = "SNPD";
static const std::string PLANNING_SERVICE = "create_motion_plan";

tesseract_common::Toolpath fromMsg(const snp_msgs::msg::ToolPaths& msg)
{
  tesseract_common::Toolpath tps;
  tps.reserve(msg.paths.size());
  for (const auto& path : msg.paths)
  {
    for (const auto& segment : path.segments)
    {
      tesseract_common::VectorIsometry3d seg;
      seg.reserve(segment.poses.size());
      for (const auto& pose : segment.poses)
      {
        Eigen::Isometry3d p;
        tf2::fromMsg(pose, p);
        seg.push_back(p);
      }
      tps.push_back(seg);
    }
  }
  return tps;
}

template <typename T>
T get(rclcpp::Node* node, const std::string& key)
{
  node->declare_parameter(key);
  T val;
  if (!node->get_parameter(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

class PlanningServer : public rclcpp::Node
{
public:
  PlanningServer()
    : rclcpp::Node("snp_planning_server")
    , env_(std::make_shared<tesseract_environment::Environment>())
    , planning_server_(std::make_shared<ProcessPlanningServer>(env_))
  {
    // TODO: Set up an environment monitor
    {
      auto urdf_string = get<std::string>(this, "robot_description");
      auto srdf_string = get<std::string>(this, "robot_description_semantic");
      auto resource_locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
      if (!env_->init(urdf_string, srdf_string, resource_locator))
        throw std::runtime_error("Failed to initialize environment");
    }

    // Register custom process planners
    planning_server_->registerProcessPlanner(TRANSITION_PLANNER, createTransitionTaskflow());
    planning_server_->registerProcessPlanner(FREESPACE_PLANNER, createFreespaceTaskflow());
    planning_server_->registerProcessPlanner(RASTER_PLANNER, createRasterTaskflow());

    // Add custom profiles
    {
      auto pd = planning_server_->getProfiles();
      pd->addProfile<SimplePlannerPlanProfile>(profile_ns::SIMPLE_DEFAULT_NAMESPACE, PROFILE,
                                               createSimplePlannerProfile());
      pd->addProfile<OMPLPlanProfile>(profile_ns::OMPL_DEFAULT_NAMESPACE, PROFILE, createOMPLProfile());
      pd->addProfile<TrajOptPlanProfile>(profile_ns::TRAJOPT_DEFAULT_NAMESPACE, PROFILE,
                                         createTrajOptToolZFreePlanProfile());
      pd->addProfile<TrajOptCompositeProfile>(profile_ns::TRAJOPT_DEFAULT_NAMESPACE, PROFILE, createTrajOptProfile());
      pd->addProfile<DescartesPlanProfile<float>>(profile_ns::DESCARTES_DEFAULT_NAMESPACE, PROFILE,
                                                  createDescartesPlanProfile<float>());
      pd->addProfile<CheckInputProfile>(profile_ns::CHECK_INPUT_DEFAULT_NAMESPACE, PROFILE,
                                        std::make_shared<CheckInputProfile>());
      pd->addProfile<SeedMinLengthProfile>(profile_ns::SEED_MIN_LENGTH_DEFAULT_NAMESPACE, PROFILE,
                                           std::make_shared<SeedMinLengthProfile>(5));
      pd->addProfile<IterativeSplineParameterizationProfile>(
          profile_ns::ITERATIVE_SPLINE_PARAMETERIZATION_DEFAULT_NAMESPACE, PROFILE,
          std::make_shared<IterativeSplineParameterizationProfile>());
    }

    // Advertise the ROS2 service
    server_ = create_service<snp_msgs::srv::GenerateMotionPlan>(
        PLANNING_SERVICE, std::bind(&PlanningServer::plan, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Started SNP motion planning server");
  }

private:
  CompositeInstruction createProgram(const ManipulatorInfo& info, const tesseract_common::Toolpath& raster_strips)
  {
    std::vector<std::string> joint_names = env_->getJointGroup(info.manipulator)->getJointNames();

    CompositeInstruction program(PROFILE, CompositeInstructionOrder::ORDERED, info);

    // Perform a freespace move to the first waypoint
    StateWaypoint swp1(joint_names, Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names.size())));
    PlanInstruction start_instruction(swp1, PlanInstructionType::START, PROFILE);
    program.setStartInstruction(start_instruction);

    for (std::size_t rs = 0; rs < raster_strips.size(); ++rs)
    {
      if (rs == 0)
      {
        // Define from start composite instruction
        CartesianWaypoint wp1 = raster_strips[rs][0];
        PlanInstruction plan_f0(wp1, PlanInstructionType::FREESPACE, PROFILE);
        plan_f0.setDescription("from_start_plan");
        CompositeInstruction from_start(PROFILE);
        from_start.setDescription("from_start");
        from_start.push_back(plan_f0);
        program.push_back(from_start);
      }

      // Define raster
      CompositeInstruction raster_segment(PROFILE);
      raster_segment.setDescription("Raster #" + std::to_string(rs + 1));

      for (std::size_t i = 1; i < raster_strips[rs].size(); ++i)
      {
        CartesianWaypoint wp = raster_strips[rs][i];
        raster_segment.push_back(PlanInstruction(wp, PlanInstructionType::LINEAR, PROFILE));
      }
      program.push_back(raster_segment);

      if (rs < raster_strips.size() - 1)
      {
        // Add transition
        CartesianWaypoint twp = raster_strips[rs + 1].front();

        PlanInstruction transition_instruction1(twp, PlanInstructionType::FREESPACE, PROFILE);
        transition_instruction1.setDescription("transition_from_end_plan");

        CompositeInstruction transition(PROFILE);
        transition.setDescription("transition_from_end");
        transition.push_back(transition_instruction1);

        program.push_back(transition);
      }
      else
      {
        // Add to end instruction
        PlanInstruction plan_f2(swp1, PlanInstructionType::FREESPACE, PROFILE);
        plan_f2.setDescription("to_end_plan");
        CompositeInstruction to_end(PROFILE);
        to_end.setDescription("to_end");
        to_end.push_back(plan_f2);
        program.push_back(to_end);
      }
    }

    return program;
  }

  void plan(const snp_msgs::srv::GenerateMotionPlan::Request::SharedPtr req,
            snp_msgs::srv::GenerateMotionPlan::Response::SharedPtr res)
  {
    try
    {
      // Create a manipulator info and program from the service request
      const std::string base_frame = req->tool_paths.paths.at(0).segments.at(0).header.frame_id;
      ManipulatorInfo manip_info(req->motion_group, base_frame, req->tcp_frame);

      ProcessPlanningRequest plan_req;
      plan_req.name = RASTER_PLANNER;
      plan_req.instructions = createProgram(manip_info, fromMsg(req->tool_paths));
      plan_req.env_state = env_->getState();

      ProcessPlanningFuture plan_result = planning_server_->run(plan_req);
      plan_result.wait();

      if (!plan_result.interface->isSuccessful())
        throw std::runtime_error("Failed to create motion plan");

      // Convert to joint trajectory
      tesseract_common::JointTrajectory jt = toJointTrajectory(plan_result.results->as<CompositeInstruction>());
      res->motion_plan = tesseract_rosutils::toMsg(jt, env_->getState());

      res->success = true;
    }
    catch (const std::exception& ex)
    {
      res->message = ex.what();
      res->success = false;
    }
  }

  tesseract_environment::Environment::Ptr env_;
  ProcessPlanningServer::Ptr planning_server_;
  rclcpp::Service<snp_msgs::srv::GenerateMotionPlan>::SharedPtr server_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto server = std::make_shared<PlanningServer>();
  rclcpp::spin(server);
  rclcpp::shutdown();
}

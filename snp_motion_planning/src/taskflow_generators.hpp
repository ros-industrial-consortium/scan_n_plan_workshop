#pragma once

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
// Task Composer
#include <tesseract_task_composer/task_composer_problem.h>
#include <tesseract_task_composer/task_composer_input.h>
#include <tesseract_task_composer/task_composer_node_names.h>
#include <tesseract_task_composer/taskflow/taskflow_task_composer_executor.h>
#include <tesseract_task_composer/profiles/contact_check_profile.h>

#include <tesseract_common/timer.h>

#include <tesseract_task_composer/nodes/motion_planner_task.h>
#include <tesseract_task_composer/nodes/min_length_task.h>
#include <tesseract_task_composer/nodes/discrete_contact_check_task.h>
#include <tesseract_task_composer/nodes/iterative_spline_parameterization_task.h>
#include <tesseract_task_composer/nodes/done_task.h>
#include <tesseract_task_composer/nodes/error_task.h>

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>

#include "raster_task/snp_raster_motion_task.h"
#include "raster_task/snp_raster_global_motion_task.h"

namespace snp_planning
{
class FreespaceMotionPipelineTask : public tesseract_planning::TaskComposerGraph
{
public:
  using Ptr = std::shared_ptr<FreespaceMotionPipelineTask>;
  using ConstPtr = std::shared_ptr<const FreespaceMotionPipelineTask>;
  using UPtr = std::unique_ptr<FreespaceMotionPipelineTask>;
  using ConstUPtr = std::unique_ptr<const FreespaceMotionPipelineTask>;

  /**
   * @brief FreespaceMotionPipelineTask
   * @details This will use the uuid as the input and output key
   * @param name The name give to the task
   */
  FreespaceMotionPipelineTask(std::string name = tesseract_planning::node_names::FREESPACE_PIPELINE_NAME)
    : tesseract_planning::TaskComposerGraph(std::move(name))
  {
    ctor(uuid_str_, uuid_str_);
  }

  FreespaceMotionPipelineTask(std::string input_key, std::string output_key,
                              std::string name = tesseract_planning::node_names::FREESPACE_PIPELINE_NAME)
    : tesseract_planning::TaskComposerGraph(std::move(name))
  {
    ctor(std::move(input_key), std::move(output_key));
  }

  ~FreespaceMotionPipelineTask() override = default;
  FreespaceMotionPipelineTask(const FreespaceMotionPipelineTask&) = delete;
  FreespaceMotionPipelineTask& operator=(const FreespaceMotionPipelineTask&) = delete;
  FreespaceMotionPipelineTask(FreespaceMotionPipelineTask&&) = delete;
  FreespaceMotionPipelineTask& operator=(FreespaceMotionPipelineTask&&) = delete;

  bool operator==(const FreespaceMotionPipelineTask& rhs) const
  {
    bool equal = true;
    equal &= TaskComposerGraph::operator==(rhs);
    return equal;
  }

  bool operator!=(const FreespaceMotionPipelineTask& rhs) const
  {
    return !operator==(rhs);
  }

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerGraph);
  }

  void ctor(std::string input_key, std::string output_key)
  {
    input_keys_.push_back(std::move(input_key));
    output_keys_.push_back(std::move(output_key));

    boost::uuids::uuid done_task = addNode(std::make_unique<tesseract_planning::DoneTask>());
    boost::uuids::uuid error_task = addNode(std::make_unique<tesseract_planning::ErrorTask>());

    // Setup Min Length Process Generator
    // This is required because trajopt requires a minimum length trajectory.
    // This is used to correct the input if it is to short.
    boost::uuids::uuid min_length_task =
        addNode(std::make_unique<tesseract_planning::MinLengthTask>(input_keys_[0], output_keys_[0]));

    // Setup OMPL
    auto ompl_planner = std::make_shared<tesseract_planning::OMPLMotionPlanner>();
    boost::uuids::uuid ompl_planner_task = addNode(
        std::make_unique<tesseract_planning::MotionPlannerTask>(ompl_planner, output_keys_[0], output_keys_[0]));

    // Setup TrajOpt
    auto trajopt_planner = std::make_shared<tesseract_planning::TrajOptMotionPlanner>();
    boost::uuids::uuid trajopt_planner_task = addNode(std::make_unique<tesseract_planning::MotionPlannerTask>(
        trajopt_planner, output_keys_[0], output_keys_[0], false));

    // Setup post collision check
    boost::uuids::uuid contact_check_task =
        addNode(std::make_unique<tesseract_planning::DiscreteContactCheckTask>(output_keys_[0]));

    // Setup time parameterization
    boost::uuids::uuid time_parameterization_task = addNode(
        std::make_unique<tesseract_planning::IterativeSplineParameterizationTask>(output_keys_[0], output_keys_[0]));

    // Add edges
    addEdges(min_length_task, { ompl_planner_task });
    addEdges(ompl_planner_task, { error_task, trajopt_planner_task });
    addEdges(trajopt_planner_task, { error_task, contact_check_task });
    addEdges(contact_check_task, { error_task, time_parameterization_task });
    addEdges(time_parameterization_task, { error_task, done_task });
  }
};

class TransitionMotionPipelineTask : public tesseract_planning::TaskComposerGraph
{
public:
  using Ptr = std::shared_ptr<TransitionMotionPipelineTask>;
  using ConstPtr = std::shared_ptr<const TransitionMotionPipelineTask>;
  using UPtr = std::unique_ptr<TransitionMotionPipelineTask>;
  using ConstUPtr = std::unique_ptr<const TransitionMotionPipelineTask>;

  /**
   * @brief TransitionMotionPipelineTask
   * @details This will use the uuid as the input and output key
   * @param name The name give to the task
   */
  TransitionMotionPipelineTask(std::string name = tesseract_planning::node_names::FREESPACE_PIPELINE_NAME)
    : tesseract_planning::TaskComposerGraph(std::move(name))
  {
    ctor(uuid_str_, uuid_str_);
  }

  TransitionMotionPipelineTask(std::string input_key, std::string output_key,
                               std::string name = tesseract_planning::node_names::FREESPACE_PIPELINE_NAME)
    : tesseract_planning::TaskComposerGraph(std::move(name))
  {
    ctor(std::move(input_key), std::move(output_key));
  }

  ~TransitionMotionPipelineTask() override = default;
  TransitionMotionPipelineTask(const TransitionMotionPipelineTask&) = delete;
  TransitionMotionPipelineTask& operator=(const TransitionMotionPipelineTask&) = delete;
  TransitionMotionPipelineTask(TransitionMotionPipelineTask&&) = delete;
  TransitionMotionPipelineTask& operator=(TransitionMotionPipelineTask&&) = delete;

  bool operator==(const TransitionMotionPipelineTask& rhs) const
  {
    bool equal = true;
    equal &= TaskComposerGraph::operator==(rhs);
    return equal;
  }

  bool operator!=(const TransitionMotionPipelineTask& rhs) const
  {
    return !operator==(rhs);
  }

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerGraph);
  }

  void ctor(std::string input_key, std::string output_key)
  {
    input_keys_.push_back(std::move(input_key));
    output_keys_.push_back(std::move(output_key));

    boost::uuids::uuid done_task = addNode(std::make_unique<tesseract_planning::DoneTask>());
    boost::uuids::uuid error_task = addNode(std::make_unique<tesseract_planning::ErrorTask>());

    // Setup Min Length Process Generator
    // This is required because trajopt requires a minimum length trajectory.
    // This is used to correct the input if it is to short.
    boost::uuids::uuid min_length_task =
        addNode(std::make_unique<tesseract_planning::MinLengthTask>(input_keys_[0], output_keys_[0]));

    auto simple_planner = std::make_shared<tesseract_planning::SimpleMotionPlanner>();
    boost::uuids::uuid simple_task = addNode(std::make_unique<tesseract_planning::MotionPlannerTask>(
        simple_planner, input_keys_[0], output_keys_[0], false));

    // Setup TrajOpt
    auto trajopt_planner = std::make_shared<tesseract_planning::TrajOptMotionPlanner>();
    boost::uuids::uuid trajopt_planner_task = addNode(std::make_unique<tesseract_planning::MotionPlannerTask>(
        trajopt_planner, output_keys_[0], output_keys_[0], false));

    // Setup post collision check
    boost::uuids::uuid contact_check_task =
        addNode(std::make_unique<tesseract_planning::DiscreteContactCheckTask>(output_keys_[0]));

    // Setup time parameterization
    boost::uuids::uuid time_parameterization_task = addNode(
        std::make_unique<tesseract_planning::IterativeSplineParameterizationTask>(output_keys_[0], output_keys_[0]));

    // Add edges
    addEdges(simple_task, { min_length_task });
    addEdges(min_length_task, { trajopt_planner_task });
    addEdges(trajopt_planner_task, { error_task, contact_check_task });
    addEdges(contact_check_task, { error_task, time_parameterization_task });
    addEdges(time_parameterization_task, { error_task, done_task });
  }
};

class CartesianMotionPipelineTask : public tesseract_planning::TaskComposerGraph
{
public:
  using Ptr = std::shared_ptr<CartesianMotionPipelineTask>;
  using ConstPtr = std::shared_ptr<const CartesianMotionPipelineTask>;
  using UPtr = std::unique_ptr<CartesianMotionPipelineTask>;
  using ConstUPtr = std::unique_ptr<const CartesianMotionPipelineTask>;

  /**
   * @brief CartesianMotionPipelineTask
   * @details This will use the uuid as the input and output key
   * @param name The name give to the task
   */
  CartesianMotionPipelineTask(std::string name = tesseract_planning::node_names::FREESPACE_PIPELINE_NAME)
    : tesseract_planning::TaskComposerGraph(std::move(name))
  {
    ctor(uuid_str_, uuid_str_);
  }

  CartesianMotionPipelineTask(std::string input_key, std::string output_key,
                              std::string name = tesseract_planning::node_names::FREESPACE_PIPELINE_NAME)
    : tesseract_planning::TaskComposerGraph(std::move(name))
  {
    ctor(std::move(input_key), std::move(output_key));
  }

  ~CartesianMotionPipelineTask() override = default;
  CartesianMotionPipelineTask(const CartesianMotionPipelineTask&) = delete;
  CartesianMotionPipelineTask& operator=(const CartesianMotionPipelineTask&) = delete;
  CartesianMotionPipelineTask(CartesianMotionPipelineTask&&) = delete;
  CartesianMotionPipelineTask& operator=(CartesianMotionPipelineTask&&) = delete;

  bool operator==(const CartesianMotionPipelineTask& rhs) const
  {
    bool equal = true;
    equal &= TaskComposerGraph::operator==(rhs);
    return equal;
  }

  bool operator!=(const CartesianMotionPipelineTask& rhs) const
  {
    return !operator==(rhs);
  }

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerGraph);
  }

  void ctor(std::string input_key, std::string output_key)
  {
    input_keys_.push_back(std::move(input_key));
    output_keys_.push_back(std::move(output_key));

    boost::uuids::uuid done_task = addNode(std::make_unique<tesseract_planning::DoneTask>());
    boost::uuids::uuid error_task = addNode(std::make_unique<tesseract_planning::ErrorTask>());

    // Setup Min Length Process Generator
    // This is required because trajopt requires a minimum length trajectory.
    // This is used to correct the input if it is to short.
    boost::uuids::uuid min_length_task =
        addNode(std::make_unique<tesseract_planning::MinLengthTask>(input_keys_[0], output_keys_[0]));

    // Setup Descartes
    //    auto descartes_planner = std::make_shared<tesseract_planning::DescartesMotionPlannerF>();
    //    boost::uuids::uuid descartes_planner_task =
    //        addNode(std::make_unique<tesseract_planning::MotionPlannerTask>(descartes_planner, output_keys_[0],
    //        output_keys_[0]));

    // Setup TrajOpt
    auto trajopt_planner = std::make_shared<tesseract_planning::TrajOptMotionPlanner>();
    boost::uuids::uuid trajopt_planner_task = addNode(std::make_unique<tesseract_planning::MotionPlannerTask>(
        trajopt_planner, output_keys_[0], output_keys_[0], false));

    // Setup post collision check
    boost::uuids::uuid contact_check_task =
        addNode(std::make_unique<tesseract_planning::DiscreteContactCheckTask>(output_keys_[0]));

    // Setup time parameterization
    boost::uuids::uuid time_parameterization_task = addNode(
        std::make_unique<tesseract_planning::IterativeSplineParameterizationTask>(output_keys_[0], output_keys_[0]));

    // Add edges
    addEdges(min_length_task, { trajopt_planner_task });
    //    addEdges(min_length_task, { descartes_planner_task });
    //    addEdges(descartes_planner_task, { error_task, trajopt_planner_task });
    addEdges(trajopt_planner_task, { error_task, contact_check_task });
    addEdges(contact_check_task, { error_task, time_parameterization_task });
    addEdges(time_parameterization_task, { error_task, done_task });
  }
};

}  // namespace snp_planning

snp_planning::RasterFtGlobalPipelineTask::UPtr createGlobalRasterPipeline()
{
  auto fs_task_gen = [](std::string description) {
    return std::make_unique<snp_planning::FreespaceMotionPipelineTask>(description);
  };
  auto trans_task_gen = [](std::string description) {
    return std::make_unique<snp_planning::TransitionMotionPipelineTask>(description);
  };
  auto raster_task_gen = [](std::string description) {
    return std::make_unique<snp_planning::CartesianMotionPipelineTask>(description);
  };
  auto global_task_gen = [](std::string input_key, std::string output_key) {
    return std::make_unique<tesseract_planning::DescartesGlobalMotionPipelineTask>(input_key, output_key);
  };
  auto rasters_task_gen = [fs_task_gen, trans_task_gen, raster_task_gen](std::string input_key,
                                                                         std::string output_key) {
    return std::make_unique<snp_planning::RasterFtMotionTask>(input_key, output_key, fs_task_gen, trans_task_gen,
                                                              raster_task_gen);
  };

  snp_planning::RasterFtGlobalPipelineTask::UPtr task = std::make_unique<snp_planning::RasterFtGlobalPipelineTask>(
      "input_program", "output_program", global_task_gen, rasters_task_gen);

  return task;
}

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(snp_planning::FreespaceMotionPipelineTask, "FreespaceMotionPipelineTask")
BOOST_CLASS_EXPORT_KEY2(snp_planning::TransitionMotionPipelineTask, "TransitionMotionPipelineTask")
BOOST_CLASS_EXPORT_KEY2(snp_planning::CartesianMotionPipelineTask, "CartesianMotionPipelineTask")

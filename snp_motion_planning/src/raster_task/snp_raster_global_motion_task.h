#ifndef SNP_TASK_COMPOSER_RASTER_FT_GLOBAL_PIPELINE_TASK_H
#define SNP_TASK_COMPOSER_RASTER_FT_GLOBAL_PIPELINE_TASK_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <boost/serialization/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_task_composer/nodes/simple_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/raster_ft_global_pipeline_task.h>
#include <tesseract_task_composer/nodes/raster_ft_motion_task.h>
#include <tesseract_task_composer/nodes/descartes_global_motion_pipeline_task.h>
#include <tesseract_task_composer/task_composer_future.h>
#include <tesseract_task_composer/task_composer_executor.h>
#include <tesseract_command_language/composite_instruction.h>

#include <tesseract_task_composer/task_composer_graph.h>
#include <tesseract_task_composer/task_composer_node_names.h>

namespace snp_planning
{
/**
 * @brief The RasterFtGlobalPipelineTask class
 * @details The required format is below.
 *
 * Composite
 * {
 *   Composite - from start
 *   Composite - Raster segment
 *   Composite - Transitions
 *   Composite - Raster segment
 *   Composite - Transitions
 *   Composite - Raster segment
 *   Composite - to end
 * }
 */
class RasterFtGlobalPipelineTask : public tesseract_planning::TaskComposerGraph
{
public:
  using Ptr = std::shared_ptr<RasterFtGlobalPipelineTask>;
  using ConstPtr = std::shared_ptr<const RasterFtGlobalPipelineTask>;
  using UPtr = std::unique_ptr<RasterFtGlobalPipelineTask>;
  using ConstUPtr = std::unique_ptr<const RasterFtGlobalPipelineTask>;

  RasterFtGlobalPipelineTask() = default;  // Required for serialization
  RasterFtGlobalPipelineTask(std::string input_key,
                             std::string output_key,
                             std::function<tesseract_planning::TaskComposerNode::UPtr(std::string, std::string)> global_task_gen = nullptr,
                             std::function<tesseract_planning::TaskComposerNode::UPtr (std::string, std::string)> rasters_task_gen = nullptr,
                             std::string name = tesseract_planning::node_names::RASTER_FT_G_PIPELINE_NAME);
  ~RasterFtGlobalPipelineTask() override = default;
  RasterFtGlobalPipelineTask(const RasterFtGlobalPipelineTask&) = delete;
  RasterFtGlobalPipelineTask& operator=(const RasterFtGlobalPipelineTask&) = delete;
  RasterFtGlobalPipelineTask(RasterFtGlobalPipelineTask&&) = delete;
  RasterFtGlobalPipelineTask& operator=(RasterFtGlobalPipelineTask&&) = delete;

  bool operator==(const RasterFtGlobalPipelineTask& rhs) const;
  bool operator!=(const RasterFtGlobalPipelineTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace snp_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(snp_planning::RasterFtGlobalPipelineTask, "RasterFtGlobalPipelineTask")

#endif  // SNP_TASK_COMPOSER_RASTER_FT_GLOBAL_PIPELINE_TASK_H

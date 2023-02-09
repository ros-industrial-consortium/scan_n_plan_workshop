#include "snp_raster_global_motion_task.h"
//#include "snp_raster_motion_task.h"

namespace snp_planning
{
RasterFtGlobalPipelineTask::RasterFtGlobalPipelineTask(std::string input_key,
                                                       std::string output_key,
                                                       std::function<tesseract_planning::TaskComposerNode::UPtr(std::string, std::string)> global_task_gen,
                                                       std::function<tesseract_planning::TaskComposerNode::UPtr(std::string, std::string)> rasters_task_gen,
                                                       std::string name)
  : TaskComposerGraph(std::move(name))
{
  if (global_task_gen == nullptr)
  {
    global_task_gen = [](std::string input_key, std::string output_key)
    {
      return std::make_unique<tesseract_planning::DescartesGlobalMotionPipelineTask>(input_key, output_key);
    };
  }
  if (rasters_task_gen == nullptr)
  {
    rasters_task_gen = [](std::string input_key, std::string output_key)
    {
      return std::make_unique<tesseract_planning::RasterFtMotionTask>(input_key, output_key);
    };
  }

  input_keys_.push_back(std::move(input_key));
  output_keys_.push_back(std::move(output_key));

  // Simple Motion Pipeline
  auto simple_task = std::make_unique<tesseract_planning::SimpleMotionPipelineTask>(input_keys_[0], output_keys_[0]);
  auto simple_uuid = addNode(std::move(simple_task));

  // Descartes global plan
  auto global_task = global_task_gen(output_keys_[0], output_keys_[0]);
  auto global_uuid = addNode(std::move(global_task));

  // Raster planner
  auto raster_task = rasters_task_gen(output_keys_[0], output_keys_[0]);
  auto raster_uuid = addNode(std::move(raster_task));

  addEdges(simple_uuid, { global_uuid });
  addEdges(global_uuid, { raster_uuid });
}

bool RasterFtGlobalPipelineTask::operator==(const RasterFtGlobalPipelineTask& rhs) const
{
  bool equal = true;
  equal &= TaskComposerGraph::operator==(rhs);
  return equal;
}
bool RasterFtGlobalPipelineTask::operator!=(const RasterFtGlobalPipelineTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void RasterFtGlobalPipelineTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerGraph);
}

}  // namespace snp_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(snp_planning::RasterFtGlobalPipelineTask)
BOOST_CLASS_EXPORT_IMPLEMENT(snp_planning::RasterFtGlobalPipelineTask)

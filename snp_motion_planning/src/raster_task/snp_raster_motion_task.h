#ifndef SNP_TASK_COMPOSER_RASTER_FT_MOTION_TASK_H
#define SNP_TASK_COMPOSER_RASTER_FT_MOTION_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <boost/serialization/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_common/any_poly.h>

#include <tesseract_task_composer/task_composer_node_info.h>
#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/task_composer_future.h>
#include <tesseract_task_composer/task_composer_executor.h>
#include <tesseract_task_composer/nodes/raster_ft_motion_task.h>
#include <tesseract_task_composer/nodes/start_task.h>
#include <tesseract_task_composer/nodes/cartesian_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/freespace_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/update_start_and_end_state_task.h>
#include <tesseract_task_composer/nodes/update_end_state_task.h>
#include <tesseract_task_composer/nodes/update_start_state_task.h>
#include <tesseract_command_language/composite_instruction.h>

namespace snp_planning
{

  class RasterFtMotionTask : public tesseract_planning::TaskComposerTask
  {

  public:
    using Ptr = std::shared_ptr<RasterFtMotionTask>;
    using ConstPtr = std::shared_ptr<const RasterFtMotionTask>;
    using UPtr = std::unique_ptr<RasterFtMotionTask>;
    using ConstUPtr = std::unique_ptr<const RasterFtMotionTask>;

    RasterFtMotionTask() = default;  // Required for serialization
    ~RasterFtMotionTask() override = default;
    RasterFtMotionTask(const RasterFtMotionTask&) = delete;
    RasterFtMotionTask& operator=(const RasterFtMotionTask&) = delete;
    RasterFtMotionTask(RasterFtMotionTask&&) = delete;
    RasterFtMotionTask& operator=(RasterFtMotionTask&&) = delete;

    RasterFtMotionTask(std::string input_key,
                       std::string output_key,
                       std::function<tesseract_planning::TaskComposerNode::UPtr(std::string)> freespace_task_gen = nullptr,
                       std::function<tesseract_planning::TaskComposerNode::UPtr(std::string)> transition_task_gen = nullptr,
                       std::function<tesseract_planning::TaskComposerNode::UPtr(std::string)> raster_task_gen = nullptr,
                       bool is_conditional = true,
                       std::string name = "SNPRasterFtMotionTask");

    tesseract_planning::TaskComposerNodeInfo::UPtr runImpl(tesseract_planning::TaskComposerInput& input,
                                                           OptionalTaskComposerExecutor executor) const;

    bool operator==(const RasterFtMotionTask& rhs) const;
    bool operator!=(const RasterFtMotionTask& rhs) const;


  protected:
    friend class tesseract_common::Serialization;
    friend class boost::serialization::access;

    static void checkTaskInput(const tesseract_common::AnyPoly& input);
    template <class Archive>
    void serialize(Archive& ar, const unsigned int /*version*/)
    {
      ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(tesseract_planning::TaskComposerTask);
    }

    std::function<tesseract_planning::TaskComposerNode::UPtr(std::string)> freespace_gen_{ nullptr };
    std::function<tesseract_planning::TaskComposerNode::UPtr(std::string)> transition_gen_{ nullptr };
    std::function<tesseract_planning::TaskComposerNode::UPtr(std::string)> raster_gen_{ nullptr };

  };

} // namespace snp_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(snp_planning::RasterFtMotionTask, "RasterFtMotionTask")

#endif  // SNP_TASK_COMPOSER_RASTER_FT_MOTION_TASK_H

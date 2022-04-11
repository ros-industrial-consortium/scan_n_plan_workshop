#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>  // TODO: replace with snp_msgs service interface
#include <tesseract_command_language/command_language.h>

tesseract_planning::CompositeInstruction createProgram(const tesseract_planning::ManipulatorInfo& manip_info,
                                                       const tesseract_common::Toolpath& raster_strips)
{
  std::string raster_profile{ "RASTER" };
  std::string transition_profile{ "TRANSITION" };
  std::string freespace_profile{ "FREESPACE" };

  // TODO: get from motion planning environment and motion group
  std::vector<std::string> joint_names{ "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

  tesseract_planning::CompositeInstruction program(raster_profile,
                                                   tesseract_planning::CompositeInstructionOrder::ORDERED, manip_info);

  tesseract_planning::StateWaypoint swp1(joint_names, Eigen::VectorXd::Zero(joint_names.size()));
  tesseract_planning::PlanInstruction start_instruction(swp1, tesseract_planning::PlanInstructionType::START,
                                                        freespace_profile);
  program.setStartInstruction(start_instruction);

  for (std::size_t rs = 0; rs < raster_strips.size(); ++rs)
  {
    if (rs == 0)
    {
      // Define from start composite instruction
      tesseract_planning::CartesianWaypoint wp1 = raster_strips[rs][0];
      tesseract_planning::PlanInstruction plan_f0(wp1, tesseract_planning::PlanInstructionType::FREESPACE,
                                                  freespace_profile);
      plan_f0.setDescription("from_start_plan");
      tesseract_planning::CompositeInstruction from_start(freespace_profile);
      from_start.setDescription("from_start");
      from_start.push_back(plan_f0);
      program.push_back(from_start);
    }

    // Define raster
    tesseract_planning::CompositeInstruction raster_segment(raster_profile);
    raster_segment.setDescription("Raster #" + std::to_string(rs + 1));

    for (std::size_t i = 1; i < raster_strips[rs].size(); ++i)
    {
      tesseract_planning::CartesianWaypoint wp = raster_strips[rs][i];
      raster_segment.push_back(
          tesseract_planning::PlanInstruction(wp, tesseract_planning::PlanInstructionType::LINEAR, raster_profile));
    }
    program.push_back(raster_segment);

    if (rs < raster_strips.size() - 1)
    {
      // Add transition
      tesseract_planning::CartesianWaypoint twp = raster_strips[rs + 1].front();

      tesseract_planning::PlanInstruction transition_instruction1(
          twp, tesseract_planning::PlanInstructionType::FREESPACE, transition_profile);
      transition_instruction1.setDescription("transition_from_end_plan");

      tesseract_planning::CompositeInstruction transition(transition_profile);
      transition.setDescription("transition_from_end");
      transition.push_back(transition_instruction1);

      program.push_back(transition);
    }
    else
    {
      // Add to end instruction
      tesseract_planning::PlanInstruction plan_f2(swp1, tesseract_planning::PlanInstructionType::FREESPACE,
                                                  freespace_profile);
      plan_f2.setDescription("to_end_plan");
      tesseract_planning::CompositeInstruction to_end(freespace_profile);
      to_end.setDescription("to_end");
      to_end.push_back(plan_f2);
      program.push_back(to_end);
    }
  }

  return program;
}

void plan(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/, std_srvs::srv::Trigger::Response::SharedPtr res)
{
  // Create a manipulator info and program from the service request
  //  tesseract_planning::ManipulatorInfo manip_info("manipulator", "floor", "buffy_tcp");
  //  tesseract_planning::CompositeInstruction program = createProgram(manip_info, tool_paths_);

  // Save the instructions to file
  //  const std::string filename = "/tmp/motion_planning_instructions.xml";
  //  tesseract_planning::Serialization::toArchiveFileXML<tesseract_planning::Instruction>(program, filename);

  res->success = true;
  res->message = "TODO: implmement planning server";
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("snp_planning_server");
  auto service = node->create_service<std_srvs::srv::Trigger>("tesseract_trigger_motion_plan", &plan);

  RCLCPP_INFO(node->get_logger(), "Started SNP motion planning server");
  rclcpp::spin(node);
  rclcpp::shutdown();
}

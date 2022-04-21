#include <boost/filesystem.hpp>
#include <rclcpp/rclcpp.hpp>
#include <open3d_interface_msgs/srv/start_yak_reconstruction.hpp>
#include <open3d_interface_msgs/srv/stop_yak_reconstruction.hpp>

static const std::string MESH_FILE_PARAMETER = "mesh_file";

class Open3dSimServer : public rclcpp::Node
{
public:
  explicit Open3dSimServer(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node(name, options)
  {
    using namespace std::placeholders;

    start_yak_srv_ = create_service<open3d_interface_msgs::srv::StartYakReconstruction>(
        "start_reconstruction", std::bind(&Open3dSimServer::startYakCB, this, _1, _2));

    stop_yak_srv_ = create_service<open3d_interface_msgs::srv::StopYakReconstruction>(
        "stop_reconstruction", std::bind(&Open3dSimServer::stopYakCB, this, _1, _2));

    declare_parameter<std::string>(MESH_FILE_PARAMETER, "");
  }

private:
  void startYakCB(const std::shared_ptr<open3d_interface_msgs::srv::StartYakReconstruction::Request> /*request*/,
                  std::shared_ptr<open3d_interface_msgs::srv::StartYakReconstruction::Response> response)
  {
    response->success = true;
    RCLCPP_INFO(get_logger(), "Yak Sim Started");
  }

  void stopYakCB(const std::shared_ptr<open3d_interface_msgs::srv::StopYakReconstruction::Request> request,
                 std::shared_ptr<open3d_interface_msgs::srv::StopYakReconstruction::Response> response)
  {
    try
    {
      // get parameter named "mesh_sourcepath" and local-var name it filepath_str
      std::string mesh_file;
      if (!get_parameter(MESH_FILE_PARAMETER, mesh_file))
        throw std::runtime_error("Failed to get parameter '" + MESH_FILE_PARAMETER + "'");

      boost::filesystem::path mesh_source_path(mesh_file);
      boost::filesystem::path mesh_target_path(request->mesh_filepath);
      boost::filesystem::copy_file(mesh_source_path, mesh_target_path,
                                   boost::filesystem::copy_option::overwrite_if_exists);
      response->success = true;
      RCLCPP_INFO_STREAM(get_logger(), "Yak simulation complete; mesh saved to '" << request->mesh_filepath << "'");
    }
    catch (const std::exception& ex)
    {
      response->success = false;
      RCLCPP_ERROR_STREAM(get_logger(), "Yak simulation failed: '" << ex.what() << "'");
    }
  }

  rclcpp::Service<open3d_interface_msgs::srv::StartYakReconstruction>::SharedPtr start_yak_srv_;
  rclcpp::Service<open3d_interface_msgs::srv::StopYakReconstruction>::SharedPtr stop_yak_srv_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Open3dSimServer>("open3d_sim_server");

  rclcpp::spin(node);
  rclcpp::shutdown();
}

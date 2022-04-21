#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <functional>
#include <boost/filesystem.hpp>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <open3d_interface_msgs/srv/start_yak_reconstruction.hpp>
#include <open3d_interface_msgs/srv/stop_yak_reconstruction.hpp>

class Open3dSimServer : public rclcpp::Node
{
public:
  explicit Open3dSimServer(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node(name, options)
  {
    using namespace std::placeholders;

    start_yak_srv_ = this->create_service<open3d_interface_msgs::srv::StartYakReconstruction>(
        "start_reconstruction", std::bind(&Open3dSimServer::startYakCB, this, _1, _2));

    stop_yak_srv_ = this->create_service<open3d_interface_msgs::srv::StopYakReconstruction>(
        "stop_reconstruction", std::bind(&Open3dSimServer::stopYakCB, this, _1, _2));

    this->declare_parameter<std::string>("mesh_sourcepath", "/meshes/part_scan.ply");
  }

private:
  void startYakCB(const std::shared_ptr<open3d_interface_msgs::srv::StartYakReconstruction::Request> /*request*/,
                  std::shared_ptr<open3d_interface_msgs::srv::StartYakReconstruction::Response> response)
  {
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Yak Sim Started");
  }

  void stopYakCB(const std::shared_ptr<open3d_interface_msgs::srv::StopYakReconstruction::Request> request,
                 std::shared_ptr<open3d_interface_msgs::srv::StopYakReconstruction::Response> response)
  {
    response->success = true;

    std::string package_path = ament_index_cpp::get_package_share_directory("snp_support");
    this->get_parameter("mesh_sourcepath",
                        filepath_str);  // get parameter named "mesh_sourcepath" and local-var name it filepath_str
    std::string mesh_sourcepath = package_path + filepath_str;

    boost::filesystem::path mesh_sourcepath_path(mesh_sourcepath);       // this is the path version of mesh_sourcepath
    boost::filesystem::path mesh_filepath_path(request->mesh_filepath);  // this is the path version of mesh_filepath
    boost::system::error_code ec;                                        // define error code

    try
    {
      boost::filesystem::copy_file(mesh_sourcepath_path, mesh_filepath_path,
                                   boost::filesystem::copy_option::overwrite_if_exists, ec);
      response->success = true;
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Yak simulation complete, mesh saved to '" << request->mesh_filepath << "'");
    }
    catch (boost::system::error_code ec)
    {
      response->success = false;
      RCLCPP_INFO_STREAM(this->get_logger(), "Yak simulation incomplete, mesh not saved.");
    }
  }
  rclcpp::Service<open3d_interface_msgs::srv::StartYakReconstruction>::SharedPtr start_yak_srv_;
  rclcpp::Service<open3d_interface_msgs::srv::StopYakReconstruction>::SharedPtr stop_yak_srv_;
  std::string filepath_str;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Open3dSimServer>("open3d_sim_server");

  rclcpp::spin(node);
  rclcpp::shutdown();
}

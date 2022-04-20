#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <functional>
#include <boost/filesystem.hpp>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
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
  }

private:
  void startYakCB(const std::shared_ptr<open3d_interface_msgs::srv::StartYakReconstruction::Request> request,
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
    std::string mesh_sourcepath = package_path + "/meshes/part_scan.ply";
    // copy this file to mesh_filepath mesh_sourcepath_
    // convert strings to file system paths
    boost::filesystem::path mesh_sourcepath_path(mesh_sourcepath);       // this is the path version of mesh_sourcepath
    boost::filesystem::path mesh_filepath_path(request->mesh_filepath);  // this is the path version of mesh_filepath

    boost::filesystem::copy_file(mesh_sourcepath_path, mesh_filepath_path,
                                 boost::filesystem::copy_option::overwrite_if_exists);

    RCLCPP_INFO_STREAM(this->get_logger(), "Yak Sim Stopped, mesh saved to " << request->mesh_filepath);
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

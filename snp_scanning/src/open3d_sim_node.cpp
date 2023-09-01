#include <boost/filesystem.hpp>
#include <rclcpp/rclcpp.hpp>
#include <industrial_reconstruction_msgs/srv/start_reconstruction.hpp>
#include <industrial_reconstruction_msgs/srv/stop_reconstruction.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>


static const std::string MESH_FILE_PARAMETER = "mesh_file";
static const std::string REFERENCE_FRAME_PARAMETER = "reference_frame";
static const std::string MESH_TOPIC = "mesh";

class Open3dSimServer : public rclcpp::Node
{
public:
  explicit Open3dSimServer(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node(name, options)
  {
    using namespace std::placeholders;

    start_srv_ = create_service<industrial_reconstruction_msgs::srv::StartReconstruction>(
        "start_reconstruction", std::bind(&Open3dSimServer::startCB, this, std::placeholders::_1, std::placeholders::_2));

    stop_srv_ = create_service<industrial_reconstruction_msgs::srv::StopReconstruction>(
        "stop_reconstruction", std::bind(&Open3dSimServer::stopCB, this, std::placeholders::_1, std::placeholders::_2));

    scan_mesh_pub_ = create_publisher<visualization_msgs::msg::Marker>(MESH_TOPIC, 10);

    declare_parameter<std::string>(MESH_FILE_PARAMETER, "");
    reference_frame = declare_parameter<std::string>(REFERENCE_FRAME_PARAMETER, "");
  }

private:
  std::string reference_frame;
  visualization_msgs::msg::Marker pclMeshToRos(const pcl::PolygonMesh& pcl_mesh) {
      visualization_msgs::msg::Marker out_msg;
      out_msg.header.frame_id = reference_frame; // Set your desired frame_id
      out_msg.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      out_msg.action = visualization_msgs::msg::Marker::ADD;
      out_msg.id = 1;
      out_msg.scale.x = 1.0;
      out_msg.scale.y = 1.0;
      out_msg.scale.z = 1.0;
      out_msg.pose.position.x = 0.0;
      out_msg.pose.position.y = 0.0;
      out_msg.pose.position.z = 0.0;
      out_msg.pose.orientation.w = 1.0;
      out_msg.pose.orientation.x = 0.0;
      out_msg.pose.orientation.y = 0.0;
      out_msg.pose.orientation.z = 0.0;

      // Access vertices and polygons from the pcl::PolygonMesh
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      pcl::fromPCLPointCloud2(pcl_mesh.cloud, cloud);
      const std::vector<pcl::Vertices>& polygons = pcl_mesh.polygons;

      for (const pcl::Vertices& polygon : polygons) {
          for (int vertex_index : polygon.vertices) {
              geometry_msgs::msg::Point curr_point;
              curr_point.x = cloud.points[vertex_index].x;
              curr_point.y = cloud.points[vertex_index].y;
              curr_point.z = cloud.points[vertex_index].z;

              std_msgs::msg::ColorRGBA curr_point_color;
              curr_point_color.r = cloud.points[vertex_index].r / 255.0;
              curr_point_color.g = cloud.points[vertex_index].g / 255.0;
              curr_point_color.b = cloud.points[vertex_index].b / 255.0;
              curr_point_color.a = 1.0;
              out_msg.points.push_back(curr_point);
              out_msg.colors.push_back(curr_point_color);
          }
      }

      return out_msg;
    }
  void startCB(const std::shared_ptr<industrial_reconstruction_msgs::srv::StartReconstruction::Request> /*request*/,
               std::shared_ptr<industrial_reconstruction_msgs::srv::StartReconstruction::Response> response)
  {
    response->success = true;
    RCLCPP_INFO(get_logger(), " Sim Started");
  }

  void stopCB(const std::shared_ptr<industrial_reconstruction_msgs::srv::StopReconstruction::Request> request,
              std::shared_ptr<industrial_reconstruction_msgs::srv::StopReconstruction::Response> response)
  {
    try
    {
      // get parameter named "mesh_sourcepath" and local-var name it filepath_str
      std::string mesh_file;
      if (!get_parameter(MESH_FILE_PARAMETER, mesh_file))
        throw std::runtime_error("Failed to get parameter '" + MESH_FILE_PARAMETER + "'");

      std::string reference_frame;
      if (!get_parameter(REFERENCE_FRAME_PARAMETER, reference_frame))
        throw std::runtime_error("Failed to get parameter '" + REFERENCE_FRAME_PARAMETER + "'");

      boost::filesystem::path mesh_source_path(mesh_file);
      boost::filesystem::path mesh_target_path(request->mesh_filepath);
      boost::filesystem::copy_file(mesh_source_path, mesh_target_path,
                                   boost::filesystem::copy_option::overwrite_if_exists);

      // Publish the mesh
      {
//        visualization_msgs::msg::Marker mesh_marker;
//        mesh_marker.header.frame_id = reference_frame;

//        mesh_marker.color.r = 200;
//        mesh_marker.color.g = 200;
//        mesh_marker.color.b = 0;
//        mesh_marker.color.a = 1;

//        mesh_marker.scale.x = 1;
//        mesh_marker.scale.y = 1;
//        mesh_marker.scale.z = 1;

//        mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
//        mesh_marker.mesh_resource = "file://" + mesh_file;

//        scan_mesh_pub_->publish(mesh_marker);


//        mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;

        // Load the PLY mesh from a file
        pcl::PolygonMesh pcl_mesh;
        pcl::io::loadPLYFile(MESH_FILE_PARAMETER, pcl_mesh);
        visualization_msgs::msg::Marker out_msg = pclMeshToRos(pcl_mesh);
        out_msg.mesh_resource = "file://" + mesh_file;
        scan_mesh_pub_->publish(out_msg);;
      }


      response->success = true;
      RCLCPP_INFO_STREAM(get_logger(),
                         "Scanning simulation complete; mesh saved to '" << request->mesh_filepath << "'");
    }
    catch (const std::exception& ex)
    {
      response->success = false;
      RCLCPP_ERROR_STREAM(get_logger(), "Scanning simulation failed: '" << ex.what() << "'");
    }
  }

  rclcpp::Service<industrial_reconstruction_msgs::srv::StartReconstruction>::SharedPtr start_srv_;
  rclcpp::Service<industrial_reconstruction_msgs::srv::StopReconstruction>::SharedPtr stop_srv_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr scan_mesh_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Open3dSimServer>("open3d_sim_server");

  rclcpp::spin(node);
  rclcpp::shutdown();
}

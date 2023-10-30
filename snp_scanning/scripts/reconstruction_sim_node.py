#!/usr/bin/env python3
import os.path

import rclpy
from rclpy.node import Node
import sys
import open3d
import numpy as np
import visualization_msgs.msg
from industrial_reconstruction_msgs.srv import StartReconstruction
from industrial_reconstruction_msgs.srv import StopReconstruction
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import shutil

MESH_FILE_PARAMETER = "mesh_file"
REFERENCE_FRAME_PARAMETER = "reference_frame"
MESH_TOPIC = "mesh"

class ReconstructionSimServer(Node):
    def __init__(self):
        super().__init__()

        self.start_srv_ = self.create_service(StartReconstruction,"start_reconstruction", self.startCB)

        self.stop_srv = self.create_service(StopReconstruction, "stop_reconstruction", self.stopCB)

        self.scan_mesh_pub_ = self.create_publisher(Marker, MESH_TOPIC, 10)

        self.mesh_file_parameter_ = self.declare_parameter(MESH_FILE_PARAMETER, rclpy.Parameter.Type.STRING, value='/home/D10.US.SWRI.ORG/dspielman/workspaces/blending_m5/swri_ws/target_ws/src/scan_n_plan_workshop/snp_automate_2022/meshes/part_scan.ply')

        self. reference_frame_parameter_= self.declare_parameter(REFERENCE_FRAME_PARAMETER, rclpy.Parameter.Type.STRING, value="base_link")

    def meshToRos(self, mesh):
        triangles = np.asarray(mesh.triangles)
        vertices = np.asarray(mesh.vertices)
        vertex_colors = np.asarray(mesh.vertex_colors)
        out_msg = visualization_msgs.msg.Marker()
        out_msg.type = out_msg.TRIANGLE_LIST
        out_msg.action = out_msg.ADD
        out_msg.id = 1
        out_msg.scale.x = 1.0
        out_msg.scale.y = 1.0
        out_msg.scale.z = 1.0
        out_msg.pose.position.x = 0.0
        out_msg.pose.position.y = 0.0
        out_msg.pose.position.z = 0.0
        out_msg.pose.orientation.w = 1.0
        out_msg.pose.orientation.x = 0.0
        out_msg.pose.orientation.y = 0.0
        out_msg.pose.orientation.z = 0.0
        for triangle in triangles:
            for vertex_index in triangle:
                curr_point = Point()
                curr_point.x = vertices[vertex_index][0]
                curr_point.y = vertices[vertex_index][1]
                curr_point.z = vertices[vertex_index][2]
                curr_point_color = ColorRGBA()
                curr_point_color.r = vertex_colors[vertex_index][0]
                curr_point_color.g = vertex_colors[vertex_index][1]
                curr_point_color.b = vertex_colors[vertex_index][2]
                curr_point_color.a = 1.0
                out_msg.points.append(curr_point)
                out_msg.colors.append(curr_point_color)
        return out_msg

    def start_cb(self, request, response):
        response.success = True
        self.get_logger().info("Sim Started")

    def stop_cb(self, request, response):
        # get parameter named "mesh_sourcepath" and local - var name it filepath_str
        try:
            mesh_file = str(self.get_parameter(MESH_FILE_PARAMETER))
            if not self.get_parameter(MESH_FILE_PARAMETER, mesh_file):
                raise Exception("Failed to get parameter '" + MESH_FILE_PARAMETER + "'")
            reference_frame = str(self.get_parameter(REFERENCE_FRAME_PARAMETER))
            if not self.get_parameter(REFERENCE_FRAME_PARAMETER, reference_frame):
                raise Exception("Failed to get parameter '" + REFERENCE_FRAME_PARAMETER + "'")

            mesh_source_path = mesh_file
            mesh_target_path = request.mesh_filepath
            shutil.copy2(mesh_source_path, mesh_target_path)

            mesh = open3d.io.read_triangle_mesh(mesh_file)
            out_msg = self.meshtoRos(mesh)
            out_msg.mesh_resource = "file://" + mesh_file
            self.scan_mesh_pub.publish(out_msg)

            response.success = True
            self.get_logger().info("Scanning simulation complete; mesh saved to '" + mesh_file + "'")

        except Exception as e:
            response.success = False
            response.message = str(e)


def main(args=str(sys.argv)):
    rclpy.init(args=args)

    node = ReconstructionSimServer("reconstruction_sim_node")
    rclpy.spin(node)
    rclpy.shutdown()




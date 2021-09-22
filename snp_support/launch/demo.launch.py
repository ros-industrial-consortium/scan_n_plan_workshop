import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'urdf/hc10.xacro'
    urdf = os.path.join(
        get_package_share_directory('snp_support'),
        urdf_file_name)

    robot_desc_config = xacro.process_file(urdf)
    robot_desc = robot_desc_config.toxml()

    rviz_file_name = 'demo.rviz'
    rviz = os.path.join(
        get_package_share_directory('snp_support'),
        rviz_file_name)

    print(rviz)
    print('-d %s' % rviz)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz],
        )
    ])
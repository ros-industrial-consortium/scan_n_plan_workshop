from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.actions import Node
import yaml
import os 


parameters = [
    {'name': 'controllers_file',       'description': 'Path to the ros2_control configuration file',  'default': ''},
]


def declare_launch_arguments():
    return [DeclareLaunchArgument(entry['name'], description=entry['description'], default_value=entry['default']) for entry in parameters]


def generate_launch_description():
    return LaunchDescription(declare_launch_arguments() + [OpaqueFunction(function=launch)])


def launch(context, *args, **kwargs):
    controllers_file = LaunchConfiguration('controllers_file')

    nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers_file],
            remappings=[
                ('~/robot_description', 'robot_description'),
            ],
            output="both",
        )
    ]

    # Load the controllers YAML file
    with open(controllers_file.perform(context), 'r') as f:
        controllers = yaml.safe_load(f)

    for key, val in controllers['controller_manager']['ros__parameters'].items():
        if type(val) is dict and type(val['type']) is str:
            nodes.append(
                Node(
                    package="controller_manager",
                    executable='spawner' if os.environ['ROS_DISTRO'] > 'foxy' else 'spawner.py',
                    arguments=[key, "-c", "/controller_manager"],
                )
            )

    return nodes

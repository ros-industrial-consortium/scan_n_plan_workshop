from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


parameters = [
    {'name': 'robot_description_file', 'description': 'Path to the URDF/xacro file',                  'default': PathJoinSubstitution([FindPackageShare("snp_automate_2022"), "urdf", "workcell.xacro",])},
    {'name': 'controllers_file',       'description': 'Path to the ros2_control configuration file',  'default': PathJoinSubstitution([FindPackageShare("snp_automate_2022"), "config", "controllers.yaml",])},
]


def declare_launch_arguments():
    return [DeclareLaunchArgument(entry['name'], description=entry['description'], default_value=entry['default']) for entry in parameters]


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ", LaunchConfiguration('robot_description_file'),])
    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, LaunchConfiguration('controllers_file')],
        output="both",
        remappings=[
            ('joint_trajectory_position_controller/follow_joint_trajectory', '/joint_trajectory_action')
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_position_controller", "-c", "/controller_manager"],
        remappings=[
            ('follow_joint_trajectory', '/joint_trajectory_action')
        ],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declare_launch_arguments() + nodes)

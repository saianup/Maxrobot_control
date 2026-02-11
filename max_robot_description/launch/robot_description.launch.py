from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
    pkg_max_robot_description = get_package_share_directory('max_robot_description')
    model_file = PathJoinSubstitution([pkg_max_robot_description, 'urdf', 'max_robot.urdf.xacro'])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(Command(['xacro ', model_file]), value_type=str)}
        ],
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    return ld

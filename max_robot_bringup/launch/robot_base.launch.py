import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. Paths to your packages
    bringup_dir = get_package_share_directory('max_robot_bringup')
    base_dir = get_package_share_directory('max_robot_base')
    description_dir = get_package_share_directory('max_robot_description')

    # 2. File Paths - Double check these filenames exist!
    # If your xacro is named differently (e.g., robot.urdf.xacro), change it here
    xacro_file = os.path.join(description_dir, 'urdf', 'max_robot.urdf.xacro')
    # This is the YAML we updated in the previous step
    controller_config = os.path.join(base_dir, 'config', 'max_robot_controllers.yaml')

    # 3. Process the URDF/Xacro
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str
        )
    }

    # 4. Robot State Publisher (Sends URDF to controller_manager)
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # 5. The Controller Manager (The main hardware driver)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="both",
    )

    # 6. Spawners (Activates the specific steering logic)
    # We spawn the joint_state_broadcaster first so we get feedback
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    bicycle_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bicycle_steering_controller"],
    )

    return LaunchDescription([
        robot_state_pub_node,
        control_node,
        jsb_spawner,
        bicycle_spawner
    ])

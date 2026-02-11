from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    pkg_max_robot_bringup = get_package_share_directory('max_robot_bringup')
    pkg_max_robot_description = get_package_share_directory('max_robot_description')

    # Launch files
    robot_base_launch_file = PathJoinSubstitution(
        [pkg_max_robot_bringup, 'launch', 'robot_base.launch.py'])
    lidar_launch_file = PathJoinSubstitution(
        [pkg_max_robot_bringup, 'launch', 'lidar.launch.py'])
    #depth_camera_launch_file = PathJoinSubstitution(
    #    [pkg_max_robot_bringup, 'launch', 'depth_camera.launch.py'])
    description_launch_file = PathJoinSubstitution(
        [pkg_max_robot_description, 'launch', 'robot_description.launch.py']
    )

    actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([robot_base_launch_file])),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([lidar_launch_file])),


            # Delay the depth camera startup for a bit
            # This prevents spiking the current on the USB by having the lidar and depth camera
            # start up at the same time as everything else
            #TimerAction(
            #    period=30.0,
            #    actions=[
            #        IncludeLaunchDescription(
            #            PythonLaunchDescriptionSource([depth_camera_launch_file])),
            #    ]
            #),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([description_launch_file])),
        ]

    max_robot_group_action = GroupAction(actions)
    ld = LaunchDescription()
    ld.add_action(max_robot_group_action)
    return ld

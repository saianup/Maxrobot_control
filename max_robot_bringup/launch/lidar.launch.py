from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os

def generate_launch_description():

    pkg_max_robot_bringup = get_package_share_directory('max_robot_bringup')
    lsm10p_config = os.path.join(pkg_max_robot_bringup, 'config', 'lsm10_p.yaml')
    laser_filter_config = os.path.join(pkg_max_robot_bringup, 'config', 'laser_filter.yaml')

    lidar_node = LifecycleNode(package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[lsm10p_config],
    )

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[laser_filter_config],
    )

    return LaunchDescription([
        lidar_node,
        laser_filter_node
    ])



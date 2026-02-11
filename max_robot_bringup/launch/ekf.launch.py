from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ekf_config_file = os.path.join(get_package_share_directory('max_robot_bringup'),'config','ekf.yaml')

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file]
    )


    return LaunchDescription([
        robot_localization_node
    ])

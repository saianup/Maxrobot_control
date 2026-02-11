import os
from pathlib import Path
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
	pkg_astra_camera_dir = get_package_share_directory('astra_camera')
	astra_camera_launch_dir = os.path.join(pkg_astra_camera_dir, 'launch')
 
	astra_S = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource(os.path.join(astra_camera_launch_dir,'astra_mini.launch.py')))

	ld = LaunchDescription()
	ld.add_action(astra_S)

	return ld

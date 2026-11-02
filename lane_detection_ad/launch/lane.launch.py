"""Real robot: lane node only (camera_ros must publish /camera).

ros2 launch lane_detection_ad lane.launch.py [start_enabled:=true] [show:=false]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory('lane_detection_ad'), 'config', 'lane_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('start_enabled', default_value='false'),
        DeclareLaunchArgument('show', default_value='true'),
        Node(
            package='lane_detection_ad', executable='detection', name='lane_detection_ad', output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'start_enabled': ParameterValue(LaunchConfiguration('start_enabled'), value_type=bool),
                'show': ParameterValue(LaunchConfiguration('show'), value_type=bool),
            }],
        ),
    ])

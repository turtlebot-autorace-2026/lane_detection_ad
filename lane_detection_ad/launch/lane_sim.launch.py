"""Gazebo (autorace_gazebo): camera bridge + lane node on simulation time, driving immediately.

Terminal 1: ros2 launch autorace_gazebo autorace.launch.py
Terminal 2: ros2 launch lane_detection_ad lane_sim.launch.py [show:=true] [v_max:=0.12]
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
        DeclareLaunchArgument('show', default_value='true'),
        DeclareLaunchArgument('start_enabled', default_value='true'),
        DeclareLaunchArgument('v_max', default_value='0.12'),
        Node(
            package='lane_detection_ad', executable='sim_camera_bridge', name='sim_camera_bridge',
            output='screen', parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='lane_detection_ad', executable='detection', name='lane_detection_ad', output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'use_sim_time': True,
                'start_enabled': ParameterValue(LaunchConfiguration('start_enabled'), value_type=bool),
                'show': ParameterValue(LaunchConfiguration('show'), value_type=bool),
                'v_max': ParameterValue(LaunchConfiguration('v_max'), value_type=float),
            }],
        ),
    ])

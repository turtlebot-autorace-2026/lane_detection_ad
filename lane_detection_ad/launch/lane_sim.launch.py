"""Gazebo (autorace_gazebo): camera bridge + lane node on simulation time, driving immediately.

Terminal 1: ros2 launch autorace_gazebo autorace.launch.py [x:= y:= yaw:=]
Terminal 2: ros2 launch lane_detection_ad lane_sim.launch.py                    # general version
            ros2 launch lane_detection_ad lane_sim.launch.py hardcoded:=true    # hardcoded version
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('lane_detection_ad')
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'lane_params.yaml')),
        DeclareLaunchArgument('start_enabled', default_value='true'),
        DeclareLaunchArgument('v_max', default_value='0.12'),
        DeclareLaunchArgument('show', default_value='true'),
        DeclareLaunchArgument('hardcoded', default_value='false'),
        DeclareLaunchArgument('start_section', default_value=''),
        Node(
            package='lane_detection_ad', executable='sim_camera_bridge', name='sim_camera_bridge',
            output='screen', parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='lane_detection_ad', executable='detection', name='lane_detection_ad', output='screen',
            parameters=[arg('params_file'), {
                'use_sim_time': True,
                'start_enabled': ParameterValue(arg('start_enabled'), value_type=bool),
                'show': ParameterValue(arg('show'), value_type=bool),
                'v_max': ParameterValue(arg('v_max'), value_type=float),
            }],
        ),
        Node(
            package='lane_detection_ad', executable='section_judge', name='section_judge', output='screen',
            parameters=[{'use_sim_time': True, 'start_section': arg('start_section')}],
            condition=IfCondition(arg('hardcoded')),
        ),
    ])

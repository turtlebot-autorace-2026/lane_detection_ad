"""Gazebo, HARDCODED version: lane_sim.launch.py + section_judge (known section order of the map).

Terminal 1: ros2 launch autorace_gazebo autorace.launch.py [x:= y:= yaw:=]
Terminal 2: ros2 launch lane_detection_ad lane_sim_hardcoded.launch.py [start_section:=after_parking]
Anchor by hand (until the mission state machine does it):
  ros2 topic pub --once /lane/section_anchor std_msgs/msg/String "{data: after_parking}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lane_detection_ad')
    return LaunchDescription([
        DeclareLaunchArgument('start_section', default_value=''),
        DeclareLaunchArgument('show', default_value='true'),
        DeclareLaunchArgument('v_max', default_value='0.12'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'lane_sim.launch.py')),
            launch_arguments={'show': LaunchConfiguration('show'), 'v_max': LaunchConfiguration('v_max')}.items(),
        ),
        Node(
            package='lane_detection_ad', executable='section_judge', name='section_judge', output='screen',
            parameters=[{'use_sim_time': True, 'start_section': LaunchConfiguration('start_section')}],
        ),
    ])

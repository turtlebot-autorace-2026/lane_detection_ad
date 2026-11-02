"""Real robot (ROS 2 Galactic): lane following, optionally with the camera node and the hardcoded section judge.

  ros2 launch lane_detection_ad lane_robot.launch.py                      # waits for /lane_state true
  ros2 launch lane_detection_ad lane_robot.launch.py start_enabled:=true  # drives immediately
  ros2 launch lane_detection_ad lane_robot.launch.py camera:=true         # also starts camera_ros publisher
  ros2 launch lane_detection_ad lane_robot.launch.py hardcoded:=true start_section:=after_parking

turtlebot3_bringup (robot.launch.py) must run separately on the robot (it provides /odom, takes /cmd_vel).
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
        DeclareLaunchArgument('start_enabled', default_value='false',
                              description='true: drive at once; false: wait for /lane_state true'),
        DeclareLaunchArgument('v_max', default_value='0.08', description='m/s, first real runs: keep low'),
        DeclareLaunchArgument('show', default_value='false', description='OpenCV window (needs a display)'),
        DeclareLaunchArgument('camera', default_value='false', description='also run camera_ros publisher'),
        DeclareLaunchArgument('hardcoded', default_value='false', description='also run section_judge'),
        DeclareLaunchArgument('start_section', default_value=''),
        Node(
            package='camera_ros', executable='publisher', name='camera_publisher', output='screen',
            condition=IfCondition(arg('camera')),
        ),
        Node(
            package='lane_detection_ad', executable='detection', name='lane_detection_ad', output='screen',
            parameters=[arg('params_file'), {
                'start_enabled': ParameterValue(arg('start_enabled'), value_type=bool),
                'show': ParameterValue(arg('show'), value_type=bool),
                'v_max': ParameterValue(arg('v_max'), value_type=float),
            }],
        ),
        Node(
            package='lane_detection_ad', executable='section_judge', name='section_judge', output='screen',
            parameters=[{'start_section': arg('start_section')}],
            condition=IfCondition(arg('hardcoded')),
        ),
    ])

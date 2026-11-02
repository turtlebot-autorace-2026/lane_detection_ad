from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    camera1_node = Node(
        package='camera_ros',
        executable='publisher',
        name='camera_publisher1',
        output='screen'
    )
    
    camera2_node = Node(
        package='camera_ros',
        executable='publisher2',
        name='camera_publisher2',
        output='screen'
    )
    
    return LaunchDescription([
        camera1_node,
        camera2_node
    ])

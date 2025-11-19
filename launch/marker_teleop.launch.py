from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_aruco_teleop',
            executable='marker_teleop',
            name='marker_teleop',
            output='screen',
        )
    ])

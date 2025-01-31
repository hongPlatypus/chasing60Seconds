from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='project1',
            executable='init_pose',
            name='init_pose',
            output='screen'  # Print log messages to the screen
        ),
        Node(
            package='project1',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen'  # Print log messages to the screen
        ),
    ])

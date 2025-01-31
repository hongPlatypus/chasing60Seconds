from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the webcam_publisher node
        Node(
            package='project1',  # Your package name
            executable='object_detection',  # Entry point defined in setup.py
            name='object_detection',  # Optional: Node name
            output='screen'  # Print log messages to the screen
        ),
        
        # Launch the object_detection_subscriber node
        Node(
            package='project1',  # Your package name
            executable='following',  # Entry point defined in setup.py
            name='following',  # Optional: Node name
            output='screen'  # Print log messages to the screen
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tb3_trajectory_following',
            executable='trajectory_publisher',
            output='screen'
        ),
        Node(
            package='tb3_trajectory_following',
            executable='pure_pursuit_controller',
            output='screen'
        )
    ])

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ---------------- TurtleBot3 Gazebo ----------------
    tb3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            )
        )
    )

    # ---------------- Trajectory Publisher ----------------
    trajectory_node = Node(
        package='tb3_trajectory_following',
        executable='trajectory_publisher',
        name='trajectory_publisher',
        output='screen'
    )

    # ---------------- Pure Pursuit Controller ----------------
    controller_node = Node(
        package='tb3_trajectory_following',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        output='screen'
    )

    # ---------------- RViz ----------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('tb3_trajectory_following'),
                'rviz',
                'tb3_pure_pursuit.rviz'
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        tb3_gazebo,
        trajectory_node,
        controller_node,
        rviz
    ])


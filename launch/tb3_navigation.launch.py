from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # ---- Paths ----
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    ddrobot_pkg = get_package_share_directory('ddrobot_nav')

    gazebo_launch = os.path.join(
        tb3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py'
    )

    rviz_config = os.path.join(
        ddrobot_pkg, 'rviz', 'ddrobot_tb3.rviz'
    )

    # ---- Gazebo ----
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch)
    )

    # ---- Nodes ----
    trajectory_pub = Node(
        package='ddrobot_nav',
        executable='trajectory_publisher',
        name='trajectory_publisher',
        output='screen'
    )

    controller = Node(
        package='ddrobot_nav',
        executable='controller_node',
        name='controller',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        # TurtleBot3 model
        SetEnvironmentVariable(
            'TURTLEBOT3_MODEL', 'burger'
        ),

        gazebo,
        trajectory_pub,
        controller,
        rviz
    ])

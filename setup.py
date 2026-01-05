from setuptools import setup

package_name = 'tb3_trajectory_following'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch',
        ['launch/trajectory_follow.launch.py']),
    ('share/tb3_trajectory_following/launch', ['launch/full_demo.launch.py']),
    ('share/tb3_trajectory_following/rviz', ['rviz/tb3_pure_pursuit.rviz']),
    ('share/' + package_name + '/data',
        ['tb3_trajectory_following/time_parameterized_trajectory.csv']),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Pure Pursuit trajectory tracking for TurtleBot3',
    license='MIT',
    entry_points={
        'console_scripts': [
            'trajectory_publisher = tb3_trajectory_following.trajectory_publisher:main',
            'pure_pursuit_controller = tb3_trajectory_following.pure_pursuit_controller:main',
        ],
    },
)

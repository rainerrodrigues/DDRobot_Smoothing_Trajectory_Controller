from setuptools import setup

package_name = 'robot_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Simple robot navigation nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'trajectory_publisher = robot_nav.trajectory_publisher:main',
            'controller_node = robot_nav.controller_node:main',
        ],
    },
)


import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'turtlebot3_control_ros2'


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*.config')
        ),
    ],
    install_requires=[
        'setuptools'
    ],
    zip_safe=True,
    maintainer='Arthur Muritiba',
    maintainer_email='anickssm@users.noreply.github.com',
    description=(
        'Monitoramento autonomo de ambiente com '
        'TurtleBot3, ROS 2 e Gazebo.'
    ),
    license='Apache-2.0',
    tests_require=[
        'pytest'
    ],
    entry_points={
        'console_scripts': [
            (
                'turtlebot_ctrl = '
                'turtlebot3_control_ros2.turtlebot_ctrl:main'
            ),
            (
                'autonomous_ctrl = '
                'turtlebot3_control_ros2.autonomous_ctrl:main'
            ),
        ],
    },
)

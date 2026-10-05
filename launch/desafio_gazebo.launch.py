#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Alguns launches oficiais do TurtleBot3 consultam estas
    # variaveis diretamente durante sua inicializacao.
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'

    control_share = get_package_share_directory(
        'turtlebot3_control_ros2'
    )

    turtlebot3_gazebo_share = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    ros_gz_sim_share = get_package_share_directory(
        'ros_gz_sim'
    )

    turtlebot3_launch_directory = os.path.join(
        turtlebot3_gazebo_share,
        'launch'
    )

    world_path = os.path.join(
        turtlebot3_gazebo_share,
        'worlds',
        'turtlebot3_dqn_stage4.world'
    )

    model_path = os.path.join(
        turtlebot3_gazebo_share,
        'models'
    )

    # Arquivo que contem a interface salva do Gazebo,
    # incluindo o plugin Visualize Lidar.
    gui_config_path = os.path.join(
        control_share,
        'config',
        'gazebo_gui.config'
    )

    set_turtlebot3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )

    set_software_rendering = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value='1'
    )

    set_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=model_path
    )

    # Servidor: mundo, fisica e sensores.
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': [
                '-r -s -v4 ',
                world_path
            ]
        }.items()
    )

    # Interface grafica com Ogre 1 e configuracao salva.
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': (
                '-g -v4 '
                '--render-engine-gui ogre '
                f'--gui-config {gui_config_path}'
            )
        }.items()
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_launch_directory,
                'robot_state_publisher.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    spawn_turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_launch_directory,
                'spawn_turtlebot3.launch.py'
            )
        ),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.0'
        }.items()
    )

    autonomous_controller = Node(
        package='turtlebot3_control_ros2',
        executable='autonomous_ctrl',
        name='autonomous_ctrl',
        output='screen',
        parameters=[
            {
                'use_sim_time': True
            }
        ]
    )

    return LaunchDescription([
        set_turtlebot3_model,
        set_software_rendering,
        set_resource_path,
        gazebo_server,
        gazebo_gui,
        robot_state_publisher,
        spawn_turtlebot3,
        autonomous_controller
    ])

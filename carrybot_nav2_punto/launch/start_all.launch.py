import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # --- 1. Gazebo con el mundo warehouse ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('carrybot_mundo'),
                'launch', 'my_world.launch.py'
            )
        )
    )

    # --- 2. Nav2 + RViz (espera 5s a que Gazebo arranque) ---
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('carrybot_nav2_punto'),
                        'launch', 'my_tb3_navigator.launch.py'
                    )
                ),
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ]
    )

    # --- 3. Publicar posición inicial (espera 20s a que Nav2 y AMCL arranquen) ---
    initial_pose = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='carrybot_nav2_punto',
                executable='initial_pose_pub',
                name='initial_pose_pub',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        nav2_launch,
        initial_pose,
    ])

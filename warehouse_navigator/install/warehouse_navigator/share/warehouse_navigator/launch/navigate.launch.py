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
                get_package_share_directory('my_world'),
                'launch', 'my_world.launch.py'
            )
        )
    )

    # --- 2. Sistema de navegación Nav2 ---
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_nav2_system'),
                'launch', 'my_tb3_navigator.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # --- 3. Nuestro nodo navegador ---
    # Se lanza con un retardo de 10 s para dar tiempo a que Nav2 arranque
    navigator_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='warehouse_navigator',
                executable='nav_to_point',
                name='warehouse_navigator',
                output='screen',
                # prefix='xterm -e' permite que el input() del usuario
                # aparezca en una ventana separada si tu entorno lo soporta
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        nav2_launch,
        navigator_node,
    ])

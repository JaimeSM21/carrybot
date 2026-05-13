"""
Este módulo genera la descripción de lanzamiento completa del sistema CarryBot.

Orquesta el arranque secuencial de los tres componentes principales:
    1. Gazebo con el mundo warehouse.
    2. Nav2 y RViz2 (tras 5 segundos para que Gazebo esté listo).
    3. Publicador de la posición inicial para AMCL (tras 20 segundos para que Nav2 esté listo).

Functions:
    generate_launch_description(): Genera y devuelve la descripción de lanzamiento completa.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Genera la descripción de lanzamiento completa del sistema CarryBot.

    Orquesta el arranque secuencial de Gazebo, Nav2 con RViz2 y el publicador
    de la posición inicial, utilizando temporizadores para respetar los tiempos
    de inicialización de cada componente.

    Returns:
        LaunchDescription: Objeto con todas las acciones de lanzamiento configuradas.

    Raises:
        PackageNotFoundError: Si los paquetes 'carrybot_mundo' o 'carrybot_nav2_punto'
            no se encuentran instalados.
        FileNotFoundError: Si alguno de los archivos de lanzamiento necesarios no existe.
    """

    # Obtener y validar los directorios de los paquetes necesarios
    try:
        carrybot_mundo_share = get_package_share_directory('carrybot_mundo')
        carrybot_nav2_share = get_package_share_directory('carrybot_nav2_punto')
    except Exception as err:
        raise Exception(
            f"No se ha encontrado uno de los paquetes necesarios: {err}"
        )

    # Validar que los archivos de lanzamiento existen antes de usarlos
    world_launch_path = os.path.join(
        carrybot_mundo_share, 'launch', 'my_world.launch.py'
    )
    if not os.path.exists(world_launch_path):
        raise FileNotFoundError(
            f"No se ha encontrado el archivo de lanzamiento del mundo en: {world_launch_path}"
        )

    nav2_launch_path = os.path.join(
        carrybot_nav2_share, 'launch', 'my_tb3_navigator.launch.py'
    )
    if not os.path.exists(nav2_launch_path):
        raise FileNotFoundError(
            f"No se ha encontrado el archivo de lanzamiento de Nav2 en: {nav2_launch_path}"
        )

    # --- 1. Gazebo con el mundo warehouse ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch_path)
    )

    # --- 2. Nav2 + RViz (espera 5s a que Gazebo arranque) ---
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_path),
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
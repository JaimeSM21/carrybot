"""
Este módulo genera la descripción de lanzamiento para la navegación autónoma del TurtleBot3.

Configura y lanza Nav2 (bringup_launch) junto con RViz2 para visualización,
cargando el mapa y los parámetros del modelo de robot indicado por la variable
de entorno TURTLEBOT3_MODEL.

Functions:
    generate_launch_description(): Genera y devuelve la descripción de lanzamiento completa.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
ROS_DISTRO = os.environ.get('ROS_DISTRO', 'jazzy')


def generate_launch_description():
    """Genera la descripción de lanzamiento para la navegación autónoma con Nav2 y RViz2.

    Carga el mapa y los parámetros de navegación según el modelo de TurtleBot3
    definido en la variable de entorno TURTLEBOT3_MODEL. Lanza el stack completo
    de Nav2 y RViz2 con la configuración del paquete carrybot_nav2_punto.

    Returns:
        LaunchDescription: Objeto con todos los argumentos y acciones de lanzamiento configurados.

    Raises:
        PackageNotFoundError: Si los paquetes 'carrybot_nav2_punto' o 'nav2_bringup' no se encuentran.
        FileNotFoundError: Si el mapa, el archivo de parámetros o la configuración de RViz no existen.
    """

    # Obtener directorios de los paquetes necesarios
    try:
        carrybot_share = get_package_share_directory('carrybot_nav2_punto')
        nav2_launch_file_dir = os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch'
        )
    except Exception as err:
        raise Exception(
            f"No se ha encontrado uno de los paquetes necesarios: {err}"
        )

    # Configuración del tiempo de simulación
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Ruta al archivo del mapa
    map_path = os.path.join(carrybot_share, 'map', 'real_map.yaml')
    if not os.path.exists(map_path):
        raise FileNotFoundError(
            f"No se ha encontrado el archivo del mapa en: {map_path}"
        )
    map_dir = LaunchConfiguration('map', default=map_path)

    # Ruta al archivo de parámetros según el modelo del robot
    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_path = os.path.join(carrybot_share, 'param', param_file_name)
    if not os.path.exists(param_path):
        raise FileNotFoundError(
            f"No se ha encontrado el archivo de parámetros para el modelo "
            f"'{TURTLEBOT3_MODEL}' en: {param_path}"
        )
    param_dir = LaunchConfiguration('params_file', default=param_path)

    # Ruta a la configuración de RViz2
    rviz_config_dir = os.path.join(
        carrybot_share, 'rviz', 'tb3_navigation2.rviz'
    )
    if not os.path.exists(rviz_config_dir):
        raise FileNotFoundError(
            f"No se ha encontrado el archivo de configuración de RViz en: {rviz_config_dir}"
        )

    return LaunchDescription([
        # Declaración de argumentos configurables desde la línea de comandos
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Lanzamiento del stack de navegación Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']
            ),
            launch_arguments={
                'map': map_dir,
                'params_file': param_dir
            }.items(),
        ),

        # Lanzamiento de RViz2 para visualización
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),
    ])
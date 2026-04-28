"""
Este módulo genera la descripción de lanzamiento para el mundo de simulación de Gazebo.

Configura y lanza el servidor de Gazebo (gzserver), el cliente de Gazebo (gzclient),
el publicador del estado del robot y el nodo de spawn del robot en el mundo warehouse.

Functions:
    generate_launch_description(): Genera y devuelve la descripción de lanzamiento completa.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Genera la descripción de lanzamiento para el mundo de simulación.

    Configura el servidor y cliente de Gazebo, el publicador del estado del robot
    y el spawn del robot en las coordenadas indicadas dentro del mundo warehouse.

    Returns:
        LaunchDescription: Objeto con todas las acciones de lanzamiento configuradas.

    Raises:
        PackageNotFoundError: Si los paquetes 'carrybot_mundo' o 'ros_gz_sim' no se encuentran.
        FileNotFoundError: Si el archivo del mundo o los archivos de lanzamiento no existen.
    """

    # Obtener directorios de los paquetes necesarios
    try:
        launch_file_dir = os.path.join(
            get_package_share_directory('carrybot_mundo'), 'launch'
        )
        ros_gz_sim = get_package_share_directory('ros_gz_sim')
    except Exception as err:
        raise PackageNotFoundError(
            f"No se ha encontrado uno de los paquetes necesarios: {err}"
        )

    # Configuración de parámetros de lanzamiento
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    # Ruta al archivo del mundo
    world = os.path.join(
        get_package_share_directory('carrybot_mundo'),
        'worlds',
        'warehouse.world'
    )

    # Comprobar que el archivo del mundo existe antes de continuar
    if not os.path.exists(world):
        raise FileNotFoundError(
            f"No se ha encontrado el archivo del mundo en la ruta: {world}"
        )

    # Configuración del servidor de Gazebo (simulación sin interfaz gráfica)
    try:
        gzserver_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-r -s -v2 ', world],
                'on_exit_shutdown': 'true'
            }.items()
        )
    except Exception as err:
        raise FileNotFoundError(
            f"No se ha podido configurar el servidor de Gazebo: {err}"
        )

    # Configuración del cliente de Gazebo (interfaz gráfica)
    try:
        gzclient_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': '-g -v2 ',
                'on_exit_shutdown': 'true'
            }.items()
        )
    except Exception as err:
        raise FileNotFoundError(
            f"No se ha podido configurar el cliente de Gazebo: {err}"
        )

    # Configuración del publicador del estado del robot (robot_state_publisher)
    try:
        robot_state_publisher_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        )
    except Exception as err:
        raise FileNotFoundError(
            f"No se ha podido configurar el robot_state_publisher: {err}"
        )

    # Configuración del spawn del robot en las coordenadas indicadas
    try:
        spawn_turtlebot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose
            }.items()
        )
    except Exception as err:
        raise FileNotFoundError(
            f"No se ha podido configurar el spawn del robot: {err}"
        )

    # Configuración de la variable de entorno para los recursos de Gazebo
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            get_package_share_directory('carrybot_mundo'),
            'models'
        )
    )

    # Construcción y devolución de la descripción de lanzamiento
    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(set_env_vars_resources)

    return ld
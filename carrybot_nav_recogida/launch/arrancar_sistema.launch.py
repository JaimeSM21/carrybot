import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Obtenemos las rutas de instalación de los paquetes
    mi_paquete_dir = get_package_share_directory('carrybot_nav_recogida')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 2. Obligamos al sistema a usar el tiempo de simulación (Gazebo)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 3. Definimos dónde están nuestros archivos de configuración
    ruta_mapa = os.path.join(mi_paquete_dir, 'maps', 'mi_almacen.yaml')
    ruta_parametros = os.path.join(mi_paquete_dir, 'param', 'nav2_params.yaml')
    ruta_rviz = os.path.join(mi_paquete_dir, 'rviz', 'visualizacion.rviz')

    # 4. Instrucción para arrancar el "equipo" completo de Nav2
    iniciar_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': ruta_mapa,
            'params_file': ruta_parametros,
            'use_sim_time': use_sim_time
        }.items()
    )

    # 5. Instrucción para abrir la interfaz visual RViz
    nodo_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', ruta_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 6. Devolvemos la lista de tareas que ROS 2 debe ejecutar de golpe
    return LaunchDescription([
        iniciar_nav2,
        nodo_rviz
    ])

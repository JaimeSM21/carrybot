import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Rutas de carpetas
    mi_paquete_dir = get_package_share_directory('carrybot_nav_recogida')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 2. DEFINICIÓN DE RUTAS REALES (Todo dentro de tu paquete)
    # Apuntamos a tu propia carpeta 'maps'
    ruta_mapa = os.path.join(mi_paquete_dir, 'maps', 'my_map.yaml')
    
    # Usamos tus propios parámetros y rviz
    ruta_parametros = os.path.join(mi_paquete_dir, 'param', 'nav2_params.yaml')
    ruta_rviz = os.path.join(mi_paquete_dir, 'rviz', 'nav2_default_view.rviz')

    # 3. Instrucción para arrancar Nav2
    iniciar_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': ruta_mapa,
            'params_file': ruta_parametros,
            'use_sim_time': use_sim_time
        }.items()
    )

    # 4. Interfaz RViz
    nodo_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', ruta_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        iniciar_nav2,
        nodo_rviz
    ])

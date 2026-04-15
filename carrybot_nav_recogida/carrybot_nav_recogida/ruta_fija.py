"""
Módulo para la navegación autónoma de recogida y entrega del proyecto CarryBot.
Este script lee coordenadas desde un archivo JSON y ejecuta una ruta secuencial.
"""

import rclpy
import time
import json
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

def crear_pose(x_coord, y_coord):
    """
    Instancia un objeto PoseStamped y asigna las coordenadas espaciales.
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x_coord)
    pose.pose.position.y = float(y_coord)
    pose.pose.orientation.w = 1.0
    return pose

def cargar_coordenadas():
    """
    Lee el archivo coordenadas.json desde el directorio de instalación del paquete.
    Devuelve un diccionario con los datos leídos.
    """
    # 1. Busca la ruta oficial donde ROS 2 ha instalado tu paquete
    paquete_dir = get_package_share_directory('carrybot_nav_recogida')
    # 2. Une la ruta con la carpeta config y el archivo
    ruta_json = os.path.join(paquete_dir, 'config', 'coordenadas.json')
    
    # 3. Abre el archivo y carga los datos
    with open(ruta_json, 'r') as archivo:
        datos = json.load(archivo)
    return datos

def main(args=None):
    """Inicializa el nodo y ejecuta el bucle de misiones leyendo desde JSON."""
    rclpy.init(args=args)
    navigator = BasicNavigator()

    # Cargar todos los datos del archivo externo
    datos_rutas = cargar_coordenadas()
    
    # Extraer el punto de entrega fijo
    coord_entrega = datos_rutas['entrega']
    meta_entrega = crear_pose(coord_entrega['x'], coord_entrega['y'])

    # Extraer los puntos de recogida y convertirlos a objetos PoseStamped
    puntos_recogida = []
    for punto in datos_rutas['recogidas']:
        puntos_recogida.append(crear_pose(punto['x'], punto['y']))

    print("--- INICIANDO BUCLE DE TRANSPORTE ---")

    for indice, meta_recogida in enumerate(puntos_recogida):
        
        print(f"\n--- MISIÓN {indice + 1} ---")
        print(f"Navegando a coordenada de recogida: X={meta_recogida.pose.position.x}, Y={meta_recogida.pose.position.y}")
        
        navigator.goToPose(meta_recogida)

        while not navigator.isTaskComplete():
            time.sleep(1)
        
        print("¡NOTIFICACIÓN! Robot en posición. Recogiendo paquete...")
        time.sleep(5) 

        print("Navegando al punto de ENTREGA fijo...")
        navigator.goToPose(meta_entrega)

        while not navigator.isTaskComplete():
            time.sleep(1)
        
        print("¡NOTIFICACIÓN! Paquete entregado.")

    print("\nTodas las misiones de la lista han sido completadas.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()

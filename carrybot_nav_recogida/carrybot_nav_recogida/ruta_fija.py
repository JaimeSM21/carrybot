import rclpy
import time
import json
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

def crear_pose(x_coord, y_coord):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x_coord)
    pose.pose.position.y = float(y_coord)
    pose.pose.orientation.w = 1.0
    return pose

def cargar_coordenadas():
    paquete_dir = get_package_share_directory('carrybot_nav_recogida')
    ruta_json = os.path.join(paquete_dir, 'config', 'coordenadas.json')
    with open(ruta_json, 'r') as archivo:
        datos = json.load(archivo)
    return datos

def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    datos_rutas = cargar_coordenadas()
    coord_entrega = datos_rutas['entrega']
    meta_entrega = crear_pose(coord_entrega['x'], coord_entrega['y'])

    print("\n--- SISTEMA DE TRANSPORTE CARRYBOT ACTIVADO ---")

    # Recorremos el diccionario de pedidos (nombre y coordenadas)
    for nombre_pedido, coords in datos_rutas['pedidos'].items():
        
        meta_recogida = crear_pose(coords['x'], coords['y'])

        #IR A RECOGER
        print(f"\n>>> [MISIÓN]: Voy a recoger el {nombre_pedido}...")
        navigator.goToPose(meta_recogida)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            time.sleep(1)
        
        print(f"¡LLEGADA! Recogiendo {nombre_pedido}. Por favor, espere 5 segundos...")
        time.sleep(5) 

        # IR A ENTREGAR
        print(f">>> [ENTREGA]: Voy de camino a entregar el {nombre_pedido}...")
        navigator.goToPose(meta_entrega)

        while not navigator.isTaskComplete():
            time.sleep(1)
        
        print(f"¡HECHO! {nombre_pedido} listo para la entrega en el punto central.\n")
        time.sleep(2)

    print("--- TODAS LAS TAREAS COMPLETADAS. VOLVIENDO A MODO ESPERA ---")
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import json
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class WarehouseNavigator(Node):
    """
    Cliente de la acción NavigateToPose de Nav2.
    Carga destinos desde un JSON y envía el robot al punto elegido por el usuario.
    """

    def __init__(self, destination: str):
        super().__init__('warehouse_navigator')

        # --- Cargar destinos desde el JSON ---
        json_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', '..', '..', '..', 'share',
            'warehouse_navigator', 'config', 'locations.json'
        )
        with open(json_path, 'r') as f:
            self.locations = json.load(f)

        # Validar que el destino existe
        if destination not in self.locations:
            self.get_logger().error(
                f"Destino '{destination}' no encontrado. "
                f"Opciones válidas: {list(self.locations.keys())}"
            )
            sys.exit(1)

        self.destination = destination
        self.goal_done = False

        # --- Cliente de acción ---
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

    def send_goal(self):
        """Espera al servidor y envía el goal de navegación."""
        self.get_logger().info('Esperando al servidor de navegación...')
        self._action_client.wait_for_server()

        loc = self.locations[self.destination]

        # Construir el mensaje goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(loc['x'])
        goal_msg.pose.pose.position.y = float(loc['y'])
        goal_msg.pose.pose.position.z = float(loc['z'])
        goal_msg.pose.pose.orientation.x = float(loc['qx'])
        goal_msg.pose.pose.orientation.y = float(loc['qy'])
        goal_msg.pose.pose.orientation.z = float(loc['qz'])
        goal_msg.pose.pose.orientation.w = float(loc['qw'])

        self.get_logger().info(
            f"Enviando robot a '{self.destination}' "
            f"-> x={loc['x']}, y={loc['y']}"
        )

        # Enviar goal con callbacks de feedback y resultado
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Callback cuando el servidor acepta o rechaza el goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal RECHAZADO por el servidor Nav2.')
            self.goal_done = True
            return

        self.get_logger().info('Goal ACEPTADO. Navegando...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        """Callback de feedback: muestra la distancia restante al destino."""
        distancia = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f"  Distancia restante al destino: {distancia:.2f} m"
        )

    def _result_callback(self, future):
        """Callback cuando la acción termina (éxito o fallo)."""
        result = future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"Robot llegó a '{self.destination}' correctamente."
            )
        else:
            self.get_logger().error(
                f"La navegación falló. Código de error: {result.result.error_code}"
            )

        self.goal_done = True


def main():
    rclpy.init()

    # --- Pedir destino al usuario por consola ---
    print("\n=== NAVEGADOR AUTÓNOMO DEL WAREHOUSE ===")
    print("Destinos disponibles:")
    print("  1) Estanteria1")
    print("  2) Estanteria2")
    print("  3) PuntoDeCarga")
    destino = input("\nEscribe el nombre del destino: ").strip()

    navigator = WarehouseNavigator(destino)
    navigator.send_goal()

    # Girar hasta que la acción termine
    while rclpy.ok() and not navigator.goal_done:
        rclpy.spin_once(navigator, timeout_sec=0.1)

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

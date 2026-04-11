"""
sector_navigator.py
-------------------
Nodo ROS 2 que:
  1. Recibe el nombre de un sector por consola.
  2. Navega hasta el primer waypoint del sector (entrada).
  3. Recorre el resto de waypoints del sector en orden (una pasada completa).

Uso:
    ros2 run carrybot_nav2_sector sector_navigator
"""

import json
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class SectorNavigator(Node):
    """
    Navega al sector indicado y lo recorre completo waypoint a waypoint.

    Estado de la máquina:
        NAVIGATING  -> enviando/esperando un goal de NavigateToPose
        DONE        -> todos los waypoints completados (o error fatal)
    """

    def __init__(self, sector: str):
        super().__init__('carrybot_nav2_sector')

        # ── Cargar fichero de sectores ──────────────────────────────────────
        json_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', '..', '..', '..', 'share',
            'carrybot_nav2_sector', 'config', 'sectors.json'
        )

        with open(json_path, 'r') as f:
            sectors = json.load(f)

        if sector not in sectors:
            self.get_logger().error(
                f"Sector '{sector}' no encontrado. "
                f"Sectores disponibles: {list(sectors.keys())}"
            )
            sys.exit(1)

        self.sector_name = sector
        self.waypoints = sectors[sector]['waypoints']   # lista de dicts
        self.current_wp_index = 0                       # índice del waypoint activo
        self.done = False                               # señal de fin para el bucle principal

        self.get_logger().info(
            f"Sector '{sector}' cargado con {len(self.waypoints)} waypoints."
        )

        # ── Cliente de acción Nav2 ──────────────────────────────────────────
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

    # ───────────────────────────── Helpers ─────────────────────────────────

    def _build_goal(self, wp: dict) -> NavigateToPose.Goal:
        """Construye el mensaje NavigateToPose.Goal a partir de un waypoint dict."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(wp['x'])
        goal_msg.pose.pose.position.y = float(wp['y'])
        goal_msg.pose.pose.position.z = float(wp['z'])
        goal_msg.pose.pose.orientation.x = float(wp['qx'])
        goal_msg.pose.pose.orientation.y = float(wp['qy'])
        goal_msg.pose.pose.orientation.z = float(wp['qz'])
        goal_msg.pose.pose.orientation.w = float(wp['qw'])

        return goal_msg

    def _label_for_index(self, index: int) -> str:
        """Etiqueta legible para el waypoint actual."""
        if index == 0:
            return f"[WP {index}] Entrada del sector '{self.sector_name}'"
        return f"[WP {index}] Recorrido del sector '{self.sector_name}'"

    # ─────────────────────────── Navegación ────────────────────────────────

    def send_next_goal(self):
        """Envía el goal correspondiente al waypoint actual."""
        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info(
                f"✓ Sector '{self.sector_name}' recorrido completamente."
            )
            self.done = True
            return

        wp = self.waypoints[self.current_wp_index]
        label = self._label_for_index(self.current_wp_index)

        self.get_logger().info(
            f"Navegando hacia {label} "
            f"-> x={wp['x']:.3f}, y={wp['y']:.3f}"
        )

        goal_msg = self._build_goal(wp)
        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

    def start(self):
        """Espera al servidor Nav2 y lanza el primer goal."""
        self.get_logger().info('Esperando al servidor de navegación Nav2...')
        self._action_client.wait_for_server()
        self.get_logger().info('Servidor Nav2 disponible. Iniciando misión.')
        self.send_next_goal()

    # ──────────────────────────── Callbacks ────────────────────────────────

    def _goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                f"Goal del waypoint {self.current_wp_index} RECHAZADO por Nav2. Abortando."
            )
            self.done = True
            return

        self.get_logger().info(
            f"Goal WP {self.current_wp_index} ACEPTADO. Navegando..."
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        distancia = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f"  [WP {self.current_wp_index}] Distancia restante: {distancia:.2f} m"
        )

    def _result_callback(self, future):
        result = future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"✓ Waypoint {self.current_wp_index} alcanzado."
            )
            self.current_wp_index += 1   # avanzar al siguiente waypoint
            self.send_next_goal()        # disparar el siguiente goal
        else:
            self.get_logger().error(
                f"✗ Fallo en waypoint {self.current_wp_index}. "
                f"Código de error: {result.result.error_code}. Abortando misión."
            )
            self.done = True


# ───────────────────────────────── Main ────────────────────────────────────

def main():
    rclpy.init()

    # ── Menú de selección de sector ────────────────────────────────────────
    print("\n=== NAVEGADOR DE SECTORES DEL WAREHOUSE ===")
    print("El robot navegará hasta el sector indicado y lo recorrerá una vez.\n")
    print("Sectores disponibles:")
    print("  · Estanteria1")
    print("  · Estanteria2")
    print("  · PuntoDeCarga")
    sector = input("\nEscribe el nombre del sector: ").strip()

    navigator = SectorNavigator(sector)
    navigator.start()

    # Girar el event-loop hasta que la misión termine
    while rclpy.ok() and not navigator.done:
        rclpy.spin_once(navigator, timeout_sec=0.1)

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

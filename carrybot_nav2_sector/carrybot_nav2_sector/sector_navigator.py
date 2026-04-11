"""
sector_navigator.py
-------------------
Nodo ROS 2 que:
  1. Recibe el nombre de un sector por consola.
  2. Envía todos los waypoints del sector de una vez usando FollowWaypoints.
  3. Nav2 se encarga de recorrerlos en orden automáticamente.

Uso:
    ros2 run carrybot_nav2_sector sector_navigator --ros-args -p use_sim_time:=true
"""

import json
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


class SectorNavigator(Node):
    """
    Envía el listado completo de waypoints de un sector a Nav2 usando FollowWaypoints.
    Nav2 los recorre en orden sin intervención adicional del nodo.
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
        self.waypoints = sectors[sector]['waypoints']
        self.done = False

        self.get_logger().info(
            f"Sector '{sector}' cargado con {len(self.waypoints)} waypoints."
        )

        # ── Cliente de acción FollowWaypoints ───────────────────────────────
        self._action_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints'
        )

    # ───────────────────────────── Helper ──────────────────────────────────

    def _build_pose(self, wp: dict) -> PoseStamped:
        """Convierte un waypoint dict en un PoseStamped."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(wp['x'])
        pose.pose.position.y = float(wp['y'])
        pose.pose.position.z = float(wp['z'])
        pose.pose.orientation.x = float(wp['qx'])
        pose.pose.orientation.y = float(wp['qy'])
        pose.pose.orientation.z = float(wp['qz'])
        pose.pose.orientation.w = float(wp['qw'])

        return pose

    # ─────────────────────────── Navegación ────────────────────────────────

    def start(self):
        """Espera al servidor Nav2 y envía todos los waypoints de una vez."""
        self.get_logger().info('Esperando al servidor follow_waypoints...')
        self._action_client.wait_for_server()
        self.get_logger().info('Servidor disponible. Enviando waypoints...')

        # Construir el goal con todos los waypoints del sector
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [self._build_pose(wp) for wp in self.waypoints]

        self.get_logger().info(
            f"Enviando {len(goal_msg.poses)} waypoints para el sector '{self.sector_name}'."
        )

        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

    # ──────────────────────────── Callbacks ────────────────────────────────

    def _goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal RECHAZADO por Nav2. Abortando.')
            self.done = True
            return

        self.get_logger().info('Goal ACEPTADO. Recorriendo sector...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        """Muestra qué waypoint está visitando Nav2 en cada momento."""
        wp_actual = feedback_msg.feedback.current_waypoint
        total = len(self.waypoints)
        self.get_logger().info(
            f"  Visitando waypoint {wp_actual + 1} de {total}..."
        )

    def _result_callback(self, future):
        result = future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"✓ Sector '{self.sector_name}' recorrido completamente."
            )
        else:
            # missed_waypoints contiene los índices de los waypoints fallidos
            missed = list(result.result.missed_waypoints)
            self.get_logger().error(
                f"✗ Misión finalizada con {len(missed)} waypoint(s) fallido(s): {missed}"
            )

        self.done = True


# ───────────────────────────────── Main ────────────────────────────────────

def main():
    rclpy.init()

    print("\n=== NAVEGADOR DE SECTORES DEL WAREHOUSE ===")
    print("El robot navegará hasta el sector indicado y lo recorrerá una vez.\n")
    print("Sectores disponibles:")
    print("  · Zona1")
    print("  · Zona2")
    sector = input("\nEscribe el nombre del sector: ").strip()

    navigator = SectorNavigator(sector)
    navigator.start()

    while rclpy.ok() and not navigator.done:
        rclpy.spin_once(navigator, timeout_sec=0.1)

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
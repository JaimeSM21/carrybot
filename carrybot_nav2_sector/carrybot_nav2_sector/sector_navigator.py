import json
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


class SectorNavigator(Node):
    """
    Cliente de la acción FollowWaypoints de Nav2.
    Carga zonas desde sectors.json y recorre todos los waypoints
    de la zona elegida por el usuario en orden.
    """

    def __init__(self, zone: str):
        super().__init__('zone_patrol')

        # --- Cargar zonas desde el JSON ---
        json_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', '..', '..', '..', 'share',
            'carrybot_nav2_sector', 'config', 'sectors.json'
        )
        with open(json_path, 'r') as f:
            self.zones = json.load(f)

        # Validar que la zona existe
        if zone not in self.zones:
            self.get_logger().error(
                f"Zona '{zone}' no encontrada. "
                f"Opciones válidas: {list(self.zones.keys())}"
            )
            raise SystemExit(1)

        self.zone = zone
        self.goal_done = False

        # --- Cliente de acción FollowWaypoints ---
        self._action_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints'
        )

    def send_goal(self):
        """Construye la lista de poses y envía el goal."""
        self.get_logger().info('Esperando al servidor de navegación...')
        self._action_client.wait_for_server()

        waypoints_data = self.zones[self.zone]['waypoints']
        description = self.zones[self.zone].get('description', '')

        self.get_logger().info(
            f"Zona: '{self.zone}' — {description}"
        )
        self.get_logger().info(
            f"Recorriendo {len(waypoints_data)} waypoints en orden..."
        )

        # Construir la lista de PoseStamped
        poses = []
        for i, wp in enumerate(waypoints_data):
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
            poses.append(pose)
            self.get_logger().info(
                f"  Waypoint {i + 1}: x={wp['x']}, y={wp['y']}"
            )

        # Construir el goal
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        # Enviar goal con callbacks
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

        self.get_logger().info('Goal ACEPTADO. Iniciando recorrido de zona...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        """Callback de feedback: muestra el waypoint actual."""
        waypoint_actual = feedback_msg.feedback.current_waypoint
        total = len(self.zones[self.zone]['waypoints'])
        self.get_logger().info(
            f"  Dirigiéndose al waypoint {waypoint_actual + 1} de {total}..."
        )

    def _result_callback(self, future):
        """Callback cuando la acción termina."""
        result = future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"Recorrido de '{self.zone}' completado correctamente."
            )
        else:
            missed = result.result.missed_waypoints
            self.get_logger().error(
                f"El recorrido terminó con errores. "
                f"Waypoints fallidos: {list(missed)}"
            )

        self.goal_done = True


def main():
    rclpy.init()

    # --- Pedir zona al usuario por consola ---
    print("\n=== PATRULLA DE ZONA DEL WAREHOUSE ===")
    print("Zonas disponibles:")
    print("  - Zona1  (primera zona del almacén)")
    print("  - Zona2  (segunda zona del almacén)")
    zona = input("\nEscribe el nombre de la zona: ").strip()

    try:
        patrol = SectorNavigator(zona)
    except SystemExit:
        return

    patrol.send_goal()

    # Girar hasta que la acción termine
    while rclpy.ok() and not patrol.goal_done:
        rclpy.spin_once(patrol, timeout_sec=0.1)

    patrol.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
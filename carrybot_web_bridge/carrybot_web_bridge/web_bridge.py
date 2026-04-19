import json
import os
import subprocess
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose, FollowWaypoints
import subprocess
import action_msgs.srv
from geometry_msgs.msg import PoseStamped, TwistStamped


class WebBridge(Node):

    def __init__(self):
        super().__init__('web_bridge')

        base = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', '..', '..', '..', 'share',
            'carrybot_web_bridge', 'config'
        )
        with open(os.path.join(base, 'locations.json')) as f:
            self.locations = json.load(f)
        with open(os.path.join(base, 'sectors.json')) as f:
            self.zones = json.load(f)
        with open(os.path.join(base, 'coordenadas.json')) as f:
            self.packages = json.load(f)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._wp_client  = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.create_subscription(String, '/web/nav_goal',     self._on_nav_goal,     10)
        self.create_subscription(String, '/web/patrol_goal',  self._on_patrol_goal,  10)
        self.create_subscription(String, '/web/ruta_fija', self._on_ruta_fija, 10)
        self.create_subscription(String, '/web/cancel', self._on_cancel, 10)
        

        self._status_pub       = self.create_publisher(String, '/web/status',            10)
        self._announcement_pub = self.create_publisher(String, '/delivery/announcement', 10)

        self.get_logger().info('WebBridge activo.')

    def _publish_status(self, msg):
        self._status_pub.publish(String(data=msg))
        self.get_logger().info(f'[STATUS] {msg}')

    def _build_pose(self, loc):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x    = float(loc['x'])
        pose.pose.position.y    = float(loc['y'])
        pose.pose.position.z    = float(loc.get('z', 0.0))
        pose.pose.orientation.x = float(loc.get('qx', 0.0))
        pose.pose.orientation.y = float(loc.get('qy', 0.0))
        pose.pose.orientation.z = float(loc.get('qz', 0.0))
        pose.pose.orientation.w = float(loc.get('qw', 1.0))
        return pose

    # ── nav_to_point ──────────────────────────────────────────────────────

    def _on_nav_goal(self, msg):
        dest = msg.data.strip()
        if dest not in self.locations:
            self._publish_status(f"ERROR: destino '{dest}' no encontrado")
            return
        self._publish_status(f"Navegando hacia '{dest}'...")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._build_pose(self.locations[dest])
        self._nav_client.wait_for_server()
        self._nav_client.send_goal_async(goal_msg).add_done_callback(
            lambda f: self._nav_response(f, dest))

    def _nav_response(self, future, dest):
        gh = future.result()
        if not gh.accepted:
            self._publish_status(f"Goal '{dest}' RECHAZADO")
            return
        gh.get_result_async().add_done_callback(
            lambda f: self._nav_result(f, dest))

    def _nav_result(self, future, dest):
        r = future.result()
        if r.status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_status(f"Robot llegó a '{dest}'")
        else:
            self._publish_status(f"Fallo navegando a '{dest}'. Código: {r.result.error_code}")

    # ── patrol_zone ───────────────────────────────────────────────────────

    def _on_patrol_goal(self, msg):
        zone = msg.data.strip()
        if zone not in self.zones:
            self._publish_status(f"ERROR: zona '{zone}' no encontrada")
            return
        wps = self.zones[zone]['waypoints']
        self._publish_status(f"Patrullando '{zone}' ({len(wps)} waypoints)...")
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [self._build_pose(wp) for wp in wps]
        self._wp_client.wait_for_server()
        self._wp_client.send_goal_async(goal_msg).add_done_callback(
            lambda f: self._patrol_response(f, zone))

    def _patrol_response(self, future, zone):
        gh = future.result()
        if not gh.accepted:
            self._publish_status(f"Patrulla '{zone}' RECHAZADA")
            return
        gh.get_result_async().add_done_callback(
            lambda f: self._patrol_result(f, zone))

    def _patrol_result(self, future, zone):
        r = future.result()
        if r.status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_status(f"Patrulla '{zone}' completada")
        else:
            self._publish_status(
                f"Patrulla '{zone}' terminada. Fallidos: {list(r.result.missed_waypoints)}")
            

    # ── ruta_fija ──────────────────────────────────────────────────────────

    def _on_ruta_fija(self, msg):
        self._publish_status("Iniciando ruta fija de todos los pedidos...")

        def run():                                          # ← dentro del método
            try:
                result = subprocess.run(
                    ['ros2', 'run', 'carrybot_nav_recogida', 'ruta_fija'],
                    capture_output=True, text=True
                )
                if result.returncode == 0:
                    self._publish_status("Ruta fija completada.")
                else:
                    self._publish_status(f"Ruta fija fallida: {result.stderr[:100]}")
            except Exception as e:
                self._publish_status(f"Error: {str(e)}")

        threading.Thread(target=run, daemon=True).start()  

    # ── Stop ──────────────────────────────────────────────────────────

    def _on_cancel(self, msg):
        """Para el robot: mata cualquier nodo de navegacion activo,
        cancela el goal en Nav2 y publica velocidad cero."""

        self.get_logger().info('Cancelando accion activa...')

        # 1. Matar cualquier nodo de navegacion que este corriendo
        for script in ['nav_to_point', 'sector_navigator', 'ruta_fija',
                    'patrol_zone', 'delivery']:
            subprocess.run(['pkill', '-f', script], capture_output=True)

        # 2. Cancelar el goal en Nav2 (navigate_to_pose)
        cancel_client = self.create_client(
            action_msgs.srv.CancelGoal,
            '/navigate_to_pose/_action/cancel_goal'
        )
        if cancel_client.wait_for_service(timeout_sec=2.0):
            cancel_client.call_async(action_msgs.srv.CancelGoal.Request())

        # 3. Cancelar el goal en Nav2 (follow_waypoints)
        cancel_wp = self.create_client(
            action_msgs.srv.CancelGoal,
            '/follow_waypoints/_action/cancel_goal'
        )
        if cancel_wp.wait_for_service(timeout_sec=2.0):
            cancel_wp.call_async(action_msgs.srv.CancelGoal.Request())

        # 4. Publicar velocidad cero para parar el robot fisicamente
        stop = TwistStamped()
        stop.header.frame_id = 'base_link'
        stop.header.stamp = self.get_clock().now().to_msg()
        stop_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        stop_pub.publish(stop)

        self._publish_status('Accion cancelada. Robot detenido.')


def main():
    rclpy.init()
    node = WebBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
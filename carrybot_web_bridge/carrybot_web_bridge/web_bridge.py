import json
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, FollowWaypoints


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
        """with open(os.path.join(base, 'paquetes.json')) as f:
            self.packages = json.load(f)"""

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._wp_client  = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.create_subscription(String, '/web/nav_goal',     self._on_nav_goal,     10)
        self.create_subscription(String, '/web/patrol_goal',  self._on_patrol_goal,  10)
        self.create_subscription(String, '/web/package_goal', self._on_package_goal, 10)

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

    # ── delivery ──────────────────────────────────────────────────────────

    def _on_package_goal(self, msg):
        pkg = msg.data.strip()
        pkg_pose = None
        for shelf, data in self.packages.items():
            if shelf == 'PuntoDeCarga':
                continue
            if pkg in data.get('paquetes', {}):
                pkg_pose = data['paquetes'][pkg]
                break
        if pkg_pose is None:
            self._publish_status(f"ERROR: paquete '{pkg}' no encontrado")
            return
        self._publish_status(f"Recogiendo '{pkg}'...")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._build_pose(pkg_pose)
        self._nav_client.wait_for_server()
        self._nav_client.send_goal_async(goal_msg).add_done_callback(
            lambda f: self._pickup_response(f, pkg))

    def _pickup_response(self, future, pkg):
        gh = future.result()
        if not gh.accepted:
            self._publish_status(f"Recogida '{pkg}' RECHAZADA")
            return
        gh.get_result_async().add_done_callback(
            lambda f: self._pickup_result(f, pkg))

    def _pickup_result(self, future, pkg):
        r = future.result()
        if r.status != GoalStatus.STATUS_SUCCEEDED:
            self._publish_status(f"Fallo al recoger '{pkg}'")
            return
        ann = String(data=f"RECOGIDA: '{pkg}' recogido. Dirigiéndose al PuntoDeCarga.")
        self._announcement_pub.publish(ann)
        self._publish_status(ann.data)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._build_pose(self.packages['PuntoDeCarga'])
        self._nav_client.send_goal_async(goal_msg).add_done_callback(
            lambda f: self._discharge_response(f, pkg))

    def _discharge_response(self, future, pkg):
        gh = future.result()
        if not gh.accepted:
            self._publish_status("Descarga RECHAZADA")
            return
        gh.get_result_async().add_done_callback(
            lambda f: self._discharge_result(f, pkg))

    def _discharge_result(self, future, pkg):
        r = future.result()
        if r.status == GoalStatus.STATUS_SUCCEEDED:
            ann = String(data=f"ENTREGA: '{pkg}' entregado correctamente.")
            self._announcement_pub.publish(ann)
            self._publish_status(ann.data)
        else:
            self._publish_status(f"Fallo al entregar '{pkg}'")


def main():
    rclpy.init()
    node = WebBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
"""
Este módulo implementa el puente de comunicación entre la interfaz web y el sistema ROS 2.

Suscribe topics publicados desde la web para controlar la navegación del robot,
y publica el estado de las operaciones de vuelta hacia la interfaz. Soporta
navegación a un punto, patrulla por zonas, ruta fija de recogida y cancelación.

Classes:
    WebBridge: Nodo ROS 2 que gestiona la comunicación entre la web y Nav2.

Functions:
    main(): Inicializa y ejecuta el nodo WebBridge.
"""

import json
import os
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
import action_msgs.srv
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose, FollowWaypoints


class WebBridge(Node):
    """Nodo ROS 2 que actúa como puente entre la interfaz web y el sistema de navegación Nav2.

    Carga la configuración de destinos, zonas y paquetes desde archivos JSON,
    y gestiona las órdenes recibidas por topics ROS 2 para controlar el robot.

    Attributes:
        locations (dict): Destinos de navegación cargados desde locations.json.
        zones (dict): Zonas de patrulla cargadas desde sectors.json.
        packages (dict): Coordenadas de paquetes cargadas desde coordenadas.json.

    Methods:
        _publish_status(msg): Publica un mensaje de estado en el topic /web/status.
        _build_pose(loc): Construye un PoseStamped a partir de un diccionario de coordenadas.
        _on_nav_goal(msg): Callback para navegar a un punto concreto.
        _on_patrol_goal(msg): Callback para patrullar una zona definida.
        _on_ruta_fija(msg): Callback para ejecutar la ruta fija de recogida.
        _on_cancel(msg): Callback para cancelar cualquier acción activa y detener el robot.
    """

    def __init__(self):
        """Inicializa el nodo, carga los archivos de configuración y crea publishers y subscribers.

        Raises:
            FileNotFoundError: Si alguno de los archivos JSON de configuración no existe.
            ValueError: Si algún archivo JSON tiene un formato incorrecto.
        """
        super().__init__('web_bridge')

        # Ruta base a los archivos de configuración del paquete
        base = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', '..', '..', '..', 'share',
            'carrybot_web_bridge', 'config'
        )

        # Cargar archivos de configuración con validación de existencia y formato
        for filename in ['locations.json', 'sectors.json', 'coordenadas.json']:
            filepath = os.path.join(base, filename)
            if not os.path.exists(filepath):
                raise FileNotFoundError(
                    f"No se ha encontrado el archivo de configuración: {filepath}"
                )

        try:
            with open(os.path.join(base, 'locations.json')) as f:
                self.locations = json.load(f)
            with open(os.path.join(base, 'sectors.json')) as f:
                self.zones = json.load(f)
            with open(os.path.join(base, 'coordenadas.json')) as f:
                self.packages = json.load(f)
        except json.JSONDecodeError as err:
            raise ValueError(
                f"Uno de los archivos de configuración JSON tiene un formato inválido: {err}"
            )

        # Clientes de acción para navegación y seguimiento de waypoints
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._wp_client  = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # Suscripciones a los topics de control enviados desde la web
        self.create_subscription(String, '/web/nav_goal',    self._on_nav_goal,    10)
        self.create_subscription(String, '/web/patrol_goal', self._on_patrol_goal, 10)
        self.create_subscription(String, '/web/ruta_fija',   self._on_ruta_fija,   10)
        self.create_subscription(String, '/web/pedido',      self._on_pedido,      10)
        self.create_subscription(String, '/web/cancel',      self._on_cancel,      10)

        # Publicadores de estado y anuncios de entrega
        self._status_pub       = self.create_publisher(String, '/web/status',            10)
        self._announcement_pub = self.create_publisher(String, '/delivery/announcement', 10)

        self.get_logger().info('WebBridge activo.')

    def _publish_status(self, msg):
        """Publica un mensaje de estado en el topic /web/status y lo registra en el logger.

        Args:
            msg (str): Mensaje de estado a publicar.
        """
        self._status_pub.publish(String(data=msg))
        self.get_logger().info(f'[STATUS] {msg}')

    def _build_pose(self, loc):
        """Construye un mensaje PoseStamped a partir de un diccionario de coordenadas.

        Args:
            loc (dict): Diccionario con las claves 'x', 'y', y opcionalmente
                'z', 'qx', 'qy', 'qz', 'qw' para posición y orientación.

        Returns:
            PoseStamped: Mensaje con la pose construida en el frame 'map'.

        Raises:
            KeyError: Si el diccionario no contiene las claves 'x' o 'y'.
            ValueError: Si alguno de los valores no puede convertirse a float.
        """
        try:
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
        except KeyError as err:
            raise KeyError(f"Falta el campo obligatorio {err} en el diccionario de coordenadas.")
        except ValueError as err:
            raise ValueError(f"Valor no numérico en el diccionario de coordenadas: {err}")

    # ── nav_to_point ──────────────────────────────────────────────────────

    def _on_nav_goal(self, msg):
        """Callback que gestiona la navegación a un destino concreto.

        Recibe el nombre del destino por el topic /web/nav_goal, lo valida
        contra el diccionario de locations y envía el goal al servidor Nav2.

        Args:
            msg (String): Mensaje ROS 2 con el nombre del destino como texto.
        """
        dest = msg.data.strip()
        if not dest:
            self._publish_status("ERROR: el nombre del destino no puede estar vacío.")
            return
        if dest not in self.locations:
            self._publish_status(f"ERROR: destino '{dest}' no encontrado.")
            return
        try:
            self._publish_status(f"Navegando hacia '{dest}'...")
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self._build_pose(self.locations[dest])
            self._nav_client.wait_for_server()
            self._nav_client.send_goal_async(goal_msg).add_done_callback(
                lambda f: self._nav_response(f, dest))
        except Exception as err:
            self._publish_status(f"ERROR al enviar goal de navegación: {err}")

    def _nav_response(self, future, dest):
        """Callback que procesa la respuesta del servidor al recibir el goal de navegación.

        Args:
            future: Objeto Future con el resultado de la petición del goal.
            dest (str): Nombre del destino al que se navega.
        """
        try:
            gh = future.result()
            if not gh.accepted:
                self._publish_status(f"Goal '{dest}' RECHAZADO.")
                return
            gh.get_result_async().add_done_callback(
                lambda f: self._nav_result(f, dest))
        except Exception as err:
            self._publish_status(f"ERROR procesando respuesta del goal '{dest}': {err}")

    def _nav_result(self, future, dest):
        """Callback que procesa el resultado final de la navegación a un punto.

        Args:
            future: Objeto Future con el resultado de la acción NavigateToPose.
            dest (str): Nombre del destino al que se navegó.
        """
        try:
            r = future.result()
            if r.status == GoalStatus.STATUS_SUCCEEDED:
                self._publish_status(f"Robot llegó a '{dest}'.")
            else:
                self._publish_status(
                    f"Fallo navegando a '{dest}'. Código: {r.result.error_code}"
                )
        except Exception as err:
            self._publish_status(f"ERROR procesando resultado de navegación a '{dest}': {err}")

    # ── patrol_zone ───────────────────────────────────────────────────────

    def _on_patrol_goal(self, msg):
        """Callback que gestiona la patrulla de una zona definida por waypoints.

        Recibe el nombre de la zona por el topic /web/patrol_goal, la valida
        contra el diccionario de zones y envía los waypoints al servidor Nav2.

        Args:
            msg (String): Mensaje ROS 2 con el nombre de la zona como texto.
        """
        zone = msg.data.strip()
        if not zone:
            self._publish_status("ERROR: el nombre de la zona no puede estar vacío.")
            return
        if zone not in self.zones:
            self._publish_status(f"ERROR: zona '{zone}' no encontrada.")
            return
        try:
            wps = self.zones[zone]['waypoints']
            self._publish_status(f"Patrullando '{zone}' ({len(wps)} waypoints)...")
            goal_msg = FollowWaypoints.Goal()
            goal_msg.poses = [self._build_pose(wp) for wp in wps]
            self._wp_client.wait_for_server()
            self._wp_client.send_goal_async(goal_msg).add_done_callback(
                lambda f: self._patrol_response(f, zone))
        except KeyError:
            self._publish_status(
                f"ERROR: la zona '{zone}' no tiene el campo 'waypoints' definido."
            )
        except Exception as err:
            self._publish_status(f"ERROR al enviar goal de patrulla: {err}")

    def _patrol_response(self, future, zone):
        """Callback que procesa la respuesta del servidor al recibir el goal de patrulla.

        Args:
            future: Objeto Future con el resultado de la petición del goal.
            zone (str): Nombre de la zona que se patrulla.
        """
        try:
            gh = future.result()
            if not gh.accepted:
                self._publish_status(f"Patrulla '{zone}' RECHAZADA.")
                return
            gh.get_result_async().add_done_callback(
                lambda f: self._patrol_result(f, zone))
        except Exception as err:
            self._publish_status(f"ERROR procesando respuesta de patrulla '{zone}': {err}")

    def _patrol_result(self, future, zone):
        """Callback que procesa el resultado final de la patrulla de una zona.

        Args:
            future: Objeto Future con el resultado de la acción FollowWaypoints.
            zone (str): Nombre de la zona patrullada.
        """
        try:
            r = future.result()
            if r.status == GoalStatus.STATUS_SUCCEEDED:
                self._publish_status(f"Patrulla '{zone}' completada.")
            else:
                self._publish_status(
                    f"Patrulla '{zone}' terminada. Fallidos: {list(r.result.missed_waypoints)}"
                )
        except Exception as err:
            self._publish_status(f"ERROR procesando resultado de patrulla '{zone}': {err}")

    # ── ruta_fija ──────────────────────────────────────────────────────────

    def _on_ruta_fija(self, msg):
        """Callback que lanza la ruta fija de recogida de todos los pedidos en un hilo separado.

        Ejecuta el nodo 'ruta_fija' del paquete 'carrybot_nav_recogida' mediante
        subprocess para no bloquear el hilo principal de ROS 2.

        Args:
            msg (String): Mensaje ROS 2 de activación (el contenido no se utiliza).
        """
        self._publish_status("Iniciando ruta fija de todos los pedidos...")

        def run():
            try:
                result = subprocess.run(
                    ['ros2', 'run', 'carrybot_nav_recogida', 'ruta_fija'],
                    capture_output=True, text=True
                )
                if result.returncode == 0:
                    self._publish_status("Ruta fija completada.")
                else:
                    self._publish_status(
                        f"Ruta fija fallida: {result.stderr[:100]}"
                    )
            except FileNotFoundError:
                self._publish_status(
                    "ERROR: no se ha encontrado el ejecutable 'ros2'. "
                    "Comprueba que ROS 2 está correctamente instalado."
                )
            except Exception as err:
                self._publish_status(f"ERROR inesperado en la ruta fija: {err}")

        threading.Thread(target=run, daemon=True).start()

    # ── pedido_individual ─────────────────────────────────────────────────────

    def _on_pedido(self, msg):
        """Callback que lanza la recogida de un pedido individual desde la web.

        Recibe el nombre del pedido por /web/pedido y ejecuta ruta_fija
        con el argumento --pedido <nombre> en un hilo separado.

        Args:
            msg (String): Nombre del pedido a ejecutar (p.ej. 'Pedido1').
        """
        nombre = msg.data.strip()
        if not nombre:
            self._publish_status("ERROR: el nombre del pedido no puede estar vacío.")
            return

        self._publish_status(f"Iniciando pedido individual: {nombre}...")

        def run():
            try:
                result = subprocess.run(
                    ['ros2', 'run', 'carrybot_nav_recogida', 'ruta_fija',
                     '--pedido', nombre],
                    capture_output=True, text=True
                )
                if result.returncode == 0:
                    self._publish_status(f"Pedido '{nombre}' completado.")
                else:
                    self._publish_status(
                        f"Pedido '{nombre}' fallido: {result.stderr[:100]}"
                    )
            except FileNotFoundError:
                self._publish_status("ERROR: ejecutable 'ros2' no encontrado.")
            except Exception as err:
                self._publish_status(f"ERROR inesperado en pedido '{nombre}': {err}")

        threading.Thread(target=run, daemon=True).start()

    # ── cancel ──────────────────────────────────────────────────────────

    def _on_cancel(self, msg):
        """Cancela cualquier acción activa y detiene físicamente el robot.

        Mata los procesos de navegación activos, cancela los goals en Nav2
        (navigate_to_pose y follow_waypoints) y publica velocidad cero.

        Args:
            msg (String): Mensaje ROS 2 de activación (el contenido no se utiliza).
        """
        self.get_logger().info('Cancelando acción activa...')

        # 1. Matar cualquier nodo de navegación que esté corriendo
        for script in ['nav_to_point', 'sector_navigator', 'ruta_fija',
                       'patrol_zone', 'delivery']:
            try:
                subprocess.run(['pkill', '-f', script], capture_output=True)
            except Exception as err:
                self.get_logger().warning(
                    f"No se pudo matar el proceso '{script}': {err}"
                )

        # 2. Cancelar el goal en Nav2 (navigate_to_pose)
        try:
            cancel_client = self.create_client(
                action_msgs.srv.CancelGoal,
                '/navigate_to_pose/_action/cancel_goal'
            )
            if cancel_client.wait_for_service(timeout_sec=2.0):
                cancel_client.call_async(action_msgs.srv.CancelGoal.Request())
            else:
                self.get_logger().warning(
                    'Servicio de cancelación de navigate_to_pose no disponible.'
                )
        except Exception as err:
            self.get_logger().error(
                f'Error al cancelar navigate_to_pose: {err}'
            )

        # 3. Cancelar el goal en Nav2 (follow_waypoints)
        try:
            cancel_wp = self.create_client(
                action_msgs.srv.CancelGoal,
                '/follow_waypoints/_action/cancel_goal'
            )
            if cancel_wp.wait_for_service(timeout_sec=2.0):
                cancel_wp.call_async(action_msgs.srv.CancelGoal.Request())
            else:
                self.get_logger().warning(
                    'Servicio de cancelación de follow_waypoints no disponible.'
                )
        except Exception as err:
            self.get_logger().error(
                f'Error al cancelar follow_waypoints: {err}'
            )

        # 4. Publicar velocidad cero para detener el robot físicamente
        try:
            stop = TwistStamped()
            stop.header.frame_id = 'base_link'
            stop.header.stamp = self.get_clock().now().to_msg()
            stop_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
            stop_pub.publish(stop)
        except Exception as err:
            self.get_logger().error(
                f'Error al publicar velocidad cero: {err}'
            )

        self._publish_status('Acción cancelada. Robot detenido.')


def main():
    """Inicializa rclpy, crea el nodo WebBridge y lo mantiene en ejecución.

    Raises:
        FileNotFoundError: Si algún archivo de configuración JSON no existe.
        ValueError: Si algún archivo JSON tiene un formato incorrecto.
        RuntimeError: Si rclpy no puede inicializarse correctamente.
    """
    try:
        rclpy.init()
        node = WebBridge()
        rclpy.spin(node)
    except (FileNotFoundError, ValueError) as err:
        print(f'Error de configuración al iniciar WebBridge: {err}')
    except RuntimeError as err:
        print(f'Error al inicializar el nodo ROS 2: {err}')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
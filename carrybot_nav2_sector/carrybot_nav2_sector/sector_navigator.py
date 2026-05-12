"""
Modulo de navegacion por sectores del warehouse.

Este modulo implementa un nodo ROS2 que permite al robot TurtleBot3
recorrer todos los waypoints de una zona del almacen de forma autonoma
usando la accion FollowWaypoints de Nav2.

Classes:
    WaypointVerificationError: Excepcion personalizada para errores de verificacion de posicion.
    SectorNavigator: Nodo ROS2 cliente de la accion FollowWaypoints.
"""

import json
import math
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseWithCovarianceStamped


# ─── Excepcion personalizada ─────────────────────────────────────────────────

class WaypointVerificationError(Exception):
    """
    Excepcion que se lanza cuando el robot no llega correctamente
    a las coordenadas esperadas de un waypoint.

    Attributes:
        waypoint_index (int): Indice del waypoint donde se produjo el fallo.
        expected_x (float): Coordenada X esperada del waypoint.
        expected_y (float): Coordenada Y esperada del waypoint.
        actual_x (float): Coordenada X real del robot al llegar.
        actual_y (float): Coordenada Y real del robot al llegar.
        tolerance (float): Tolerancia maxima permitida en metros.
    """

    def __init__(self, waypoint_index, expected_x, expected_y,
                 actual_x, actual_y, tolerance):
        """
        Inicializa la excepcion con los datos del waypoint fallido.

        Args:
            waypoint_index (int): Indice del waypoint donde se produjo el fallo.
            expected_x (float): Coordenada X esperada del waypoint.
            expected_y (float): Coordenada Y esperada del waypoint.
            actual_x (float): Coordenada X real del robot al llegar.
            actual_y (float): Coordenada Y real del robot al llegar.
            tolerance (float): Tolerancia maxima permitida en metros.
        """
        self.waypoint_index = waypoint_index
        self.expected_x = expected_x
        self.expected_y = expected_y
        self.actual_x = actual_x
        self.actual_y = actual_y
        self.tolerance = tolerance
        distancia = math.sqrt(
            (actual_x - expected_x) ** 2 + (actual_y - expected_y) ** 2
        )
        super().__init__(
            f"Waypoint {waypoint_index}: el robot esta a {distancia:.2f} m "
            f"del objetivo (tolerancia: {tolerance} m). "
            f"Esperado: ({expected_x:.2f}, {expected_y:.2f}), "
            f"Real: ({actual_x:.2f}, {actual_y:.2f})"
        )


# ─── Nodo principal ───────────────────────────────────────────────────────────

class SectorNavigator(Node):
    """
    Nodo ROS2 cliente de la accion FollowWaypoints de Nav2.

    Carga las zonas del almacen desde un fichero sectors.json y recorre
    todos los waypoints de la zona elegida por el usuario en orden.
    Verifica que el robot llega correctamente a cada waypoint mediante
    la odometria publicada en el topic /odom.

    Attributes:
        zones (dict): Diccionario con las zonas y sus waypoints cargado del JSON.
        zone (str): Nombre de la zona que se va a recorrer.
        goal_done (bool): Indica si la accion ha terminado.
        tolerance (float): Tolerancia en metros para verificar la posicion del robot.
        current_position (tuple): Ultima posicion conocida del robot (x, y).

    Methods:
        send_goal(): Construye la lista de poses y envia el goal a Nav2.
        verify_waypoint_position(wp_index, expected_x, expected_y): Verifica
            que el robot esta en las coordenadas del waypoint.
    """

    # Tolerancia maxima en metros para la verificacion de waypoints
    TOLERANCE = 0.5

    def __init__(self, zone: str):
        """
        Inicializa el nodo SectorNavigator con la zona indicada.

        Args:
            zone (str): Nombre de la zona a recorrer (debe existir en sectors.json).

        Raises:
            SystemExit: Si la zona indicada no existe en el fichero JSON.
            FileNotFoundError: Si el fichero sectors.json no se encuentra.
            json.JSONDecodeError: Si el fichero sectors.json tiene un formato invalido.
        """
        super().__init__('zone_patrol')

        # --- Cargar zonas desde el JSON ---
        json_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', '..', '..', '..', 'share',
            'carrybot_nav2_sector', 'config', 'sectors.json'
        )

        try:
            with open(json_path, 'r') as f:
                self.zones = json.load(f)
        except FileNotFoundError:
            self.get_logger().error(
                f"Fichero sectors.json no encontrado en: {json_path}"
            )
            raise
        except json.JSONDecodeError as err:
            self.get_logger().error(
                f"Error al parsear sectors.json: {err}"
            )
            raise

        # Validar que la zona existe
        if zone not in self.zones:
            self.get_logger().error(
                f"Zona '{zone}' no encontrada. "
                f"Opciones validas: {list(self.zones.keys())}"
            )
            raise SystemExit(1)

        self.zone = zone
        self.goal_done = False
        self.current_position = (0.0, 0.0)
        self.current_waypoint_index = 0

        # --- Suscripcion a odometria para verificar posicion ---
        self._odom_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._odom_callback, 10
        )

        # --- Cliente de accion FollowWaypoints ---
        self._action_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints'
        )

    # ─── Callbacks ───────────────────────────────────────────────────────────

    def _odom_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback que actualiza la posicion actual del robot desde /amcl_pose.

        Args:
            msg (PoseWithCovarianceStamped): Estimacion de posicion de AMCL en frame map.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_position = (x, y)

    def _goal_response_callback(self, future):
        """
        Callback que se ejecuta cuando el servidor acepta o rechaza el goal.

        Args:
            future: Future con el resultado de la peticion del goal.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal RECHAZADO por el servidor Nav2.')
            self.goal_done = True
            return

        self.get_logger().info('Goal ACEPTADO. Iniciando recorrido de zona...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        """
        Callback de feedback que muestra el waypoint actual y verifica
        la posicion del robot en el momento exacto en que el indice sube.

        La verificacion se dispara cuando current_waypoint aumenta: en ese
        instante Nav2 ya ha declarado el waypoint anterior como alcanzado y
        la posicion de /amcl_pose refleja donde quedo el robot al terminar.
        No se usa ningun timer porque el cambio de indice ya es la senal
        de que el waypoint fue procesado por Nav2.

        Args:
            feedback_msg: Mensaje de feedback con current_waypoint.
        """
        waypoint_actual = feedback_msg.feedback.current_waypoint
        total = len(self.zones[self.zone]['waypoints'])

        self.get_logger().info(
            f"  Dirigiendose al waypoint {waypoint_actual + 1} de {total}..."
        )

        # El indice subio: el waypoint anterior acaba de ser procesado por Nav2.
        # Verificamos la posicion en este momento exacto.
        if waypoint_actual > self.current_waypoint_index:
            wp_completed = self.current_waypoint_index
            wp_data = self.zones[self.zone]['waypoints'][wp_completed]

            try:
                self.verify_waypoint_position(
                    wp_index=wp_completed,
                    expected_x=float(wp_data['x']),
                    expected_y=float(wp_data['y'])
                )
                self.get_logger().info(
                    f"  Verificacion OK: waypoint {wp_completed + 1} alcanzado."
                )
            except WaypointVerificationError as err:
                self.get_logger().warn(f"  ADVERTENCIA de posicion: {err}")

            # Actualizar el indice DESPUES de verificar, no antes
            self.current_waypoint_index = waypoint_actual

    def _result_callback(self, future):
        """
        Callback que se ejecuta cuando la accion FollowWaypoints termina.

        Args:
            future: Future con el resultado final de la accion.
        """
        result = future.result()

        try:
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f"Recorrido de '{self.zone}' completado correctamente."
                )
            else:
                missed = result.result.missed_waypoints
                self.get_logger().error(
                    f"El recorrido termino con errores. "
                    f"Waypoints fallidos: {list(missed)}"
                )
        except Exception as err:
            self.get_logger().error(
                f"Status recibido: {result.status}  "
                f"(2=CANCELED, 4=ABORTED, 6=SUCCEEDED)"
            )
            self.get_logger().error(
                f"Error inesperado al procesar el resultado: "
                f"{type(err).__name__}: {err}"
            )
        finally:
            self.goal_done = True

    # ─── Metodos publicos ─────────────────────────────────────────────────────

    def verify_waypoint_position(self, wp_index: int,
                                  expected_x: float,
                                  expected_y: float):
        """
        Verifica que el robot esta en las coordenadas del waypoint completado.

        Calcula la distancia euclidea entre la posicion actual del robot
        (obtenida de /odom) y las coordenadas esperadas del waypoint.
        Si la distancia supera la tolerancia, lanza WaypointVerificationError.

        Args:
            wp_index (int): Indice del waypoint a verificar (0-based).
            expected_x (float): Coordenada X esperada del waypoint en metros.
            expected_y (float): Coordenada Y esperada del waypoint en metros.

        Returns:
            float: Distancia en metros entre la posicion real y la esperada.

        Raises:
            WaypointVerificationError: Si la distancia al waypoint supera
                la tolerancia establecida (TOLERANCE).
        """
        actual_x, actual_y = self.current_position
        distancia = math.sqrt(
            (actual_x - expected_x) ** 2 + (actual_y - expected_y) ** 2
        )

        if distancia > self.TOLERANCE:
            raise WaypointVerificationError(
                waypoint_index=wp_index + 1,
                expected_x=expected_x,
                expected_y=expected_y,
                actual_x=actual_x,
                actual_y=actual_y,
                tolerance=self.TOLERANCE
            )

        return distancia

    def send_goal(self):
        """
        Construye la lista de poses del sector y envia el goal a Nav2.

        Espera a que el servidor de accion FollowWaypoints este disponible,
        construye los mensajes PoseStamped para cada waypoint de la zona
        y envia el goal con los callbacks de feedback y resultado.
        """
        self.get_logger().info('Esperando al servidor de navegacion...')
        self._action_client.wait_for_server()

        waypoints_data = self.zones[self.zone]['waypoints']
        description = self.zones[self.zone].get('description', '')

        self.get_logger().info(f"Zona: '{self.zone}' — {description}")
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

        # Construir y enviar el goal
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    """
    Funcion principal de ejecucion del nodo SectorNavigator.

    Inicializa rclpy, solicita al usuario la zona a recorrer por consola,
    crea el nodo y mantiene el bucle de spin hasta que la accion termina.
    """
    rclpy.init()

    print("\n=== PATRULLA DE ZONA DEL WAREHOUSE ===")
    print("Zonas disponibles:")
    print("  - Zona1  (primera zona del almacen)")
    print("  - Zona2  (segunda zona del almacen)")
    zona = input("\nEscribe el nombre de la zona: ").strip()

    try:
        patrol = SectorNavigator(zona)
    except FileNotFoundError:
        print("Error: no se encontro el fichero de zonas. Verifica la instalacion del paquete.")
        return
    except json.JSONDecodeError:
        print("Error: el fichero de zonas tiene un formato invalido.")
        return
    except SystemExit:
        return

    try:
        patrol.send_goal()

        while rclpy.ok() and not patrol.goal_done:
            rclpy.spin_once(patrol, timeout_sec=0.1)

    except KeyboardInterrupt:
        print("\nNavegacion interrumpida por el usuario.")
    finally:
        patrol.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
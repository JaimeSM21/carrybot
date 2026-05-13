"""
Este módulo implementa un cliente de navegación autónoma para un robot en ROS 2.

Carga destinos desde un archivo JSON de configuración y envía el robot al punto
elegido por el usuario mediante la acción NavigateToPose de Nav2.

Classes:
    WarehouseNavigator: Nodo ROS 2 cliente de la acción NavigateToPose.

Functions:
    main(): Inicializa rclpy, solicita el destino al usuario y ejecuta la navegación.
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


class WarehouseNavigator(Node):
    """Cliente de la acción NavigateToPose de Nav2.

    Carga destinos desde un archivo JSON y envía el robot al punto
    elegido por el usuario. Gestiona las respuestas del servidor de
    navegación mediante callbacks asíncronos.

    Attributes:
        locations (dict): Diccionario de destinos cargado desde el JSON.
        destination (str): Nombre del destino seleccionado por el usuario.
        goal_done (bool): Indica si la acción de navegación ha finalizado.

    Methods:
        send_goal(): Envía el goal de navegación al servidor Nav2.
    """

    def __init__(self, destination: str):
        """Inicializa el nodo, carga los destinos y valida el destino solicitado.

        Args:
            destination (str): Nombre del destino al que debe navegar el robot.

        Raises:
            FileNotFoundError: Si el archivo locations.json no se encuentra en la ruta esperada.
            ValueError: Si el contenido del JSON no tiene el formato correcto.
            SystemExit: Si el destino indicado no existe en el archivo de configuración.
        """
        super().__init__('carrybot_nav2_punto')

        # --- Cargar destinos desde el JSON ---
        json_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', '..', '..', '..', 'share',
            'carrybot_nav2_punto', 'config', 'locations.json'
        )

        if not os.path.exists(json_path):
            raise FileNotFoundError(
                f"No se ha encontrado el archivo de configuración en: {json_path}"
            )

        try:
            with open(json_path, 'r') as f:
                self.locations = json.load(f)
        except json.JSONDecodeError as err:
            raise ValueError(
                f"El archivo locations.json no tiene un formato JSON válido: {err}"
            )

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
        """Espera al servidor Nav2 y envía el goal de navegación al destino configurado.

        Construye el mensaje PoseStamped con las coordenadas del destino cargadas
        desde el JSON y lo envía de forma asíncrona con callbacks de feedback y resultado.

        Raises:
            KeyError: Si algún campo de posición u orientación falta en el JSON del destino.
            ValueError: Si los valores de posición u orientación no pueden convertirse a float.
        """
        self.get_logger().info('Esperando al servidor de navegación...')
        self._action_client.wait_for_server()

        try:
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

        except KeyError as err:
            raise KeyError(
                f"El campo {err} no existe en la configuración del destino '{self.destination}'."
            )
        except ValueError as err:
            raise ValueError(
                f"Valor no numérico en la configuración del destino '{self.destination}': {err}"
            )

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
        """Callback que se ejecuta cuando el servidor acepta o rechaza el goal.

        Args:
            future: Objeto Future con el resultado de la petición del goal.
        """
        try:
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().error('Goal RECHAZADO por el servidor Nav2.')
                self.goal_done = True
                return

            self.get_logger().info('Goal ACEPTADO. Navegando...')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._result_callback)

        except Exception as err:
            self.get_logger().error(
                f'Error al procesar la respuesta del goal: {err}'
            )
            self.goal_done = True

    def _feedback_callback(self, feedback_msg):
        """Callback de feedback que muestra la distancia restante al destino.

        Args:
            feedback_msg: Mensaje de feedback con el campo distance_remaining.
        """
        try:
            distancia = feedback_msg.feedback.distance_remaining
            self.get_logger().info(
                f"  Distancia restante al destino: {distancia:.2f} m"
            )
        except Exception as err:
            self.get_logger().error(
                f'Error al procesar el feedback de navegación: {err}'
            )

    def _result_callback(self, future):
        """Callback que se ejecuta cuando la acción de navegación termina.

        Comprueba si el robot llegó al destino correctamente o si la
        navegación falló, registrando el resultado en el logger.

        Args:
            future: Objeto Future con el resultado final de la acción.
        """
        try:
            result = future.result()

            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f"Robot llegó a '{self.destination}' correctamente."
                )
            else:
                self.get_logger().error(
                    f"La navegación falló. Código de error: {result.result.error_code}"
                )

        except Exception as err:
            self.get_logger().error(
                f'Error al procesar el resultado de la navegación: {err}'
            )
        finally:
            self.goal_done = True


def main():
    """Inicializa rclpy, solicita el destino al usuario y ejecuta la navegación.

    Muestra los destinos disponibles por consola, crea el nodo WarehouseNavigator
    y lo mantiene en ejecución hasta que la acción de navegación finaliza.

    Raises:
        FileNotFoundError: Si el archivo locations.json no se encuentra.
        ValueError: Si el JSON de configuración tiene un formato incorrecto.
        RuntimeError: Si rclpy no puede inicializarse correctamente.
    """
    try:
        rclpy.init()

        # --- Pedir destino al usuario por consola ---
        print("\n=== NAVEGADOR AUTÓNOMO DEL WAREHOUSE ===")
        print("Destinos disponibles:")
        print("  1) Estanteria1")
        print("  2) Estanteria2")
        print("  3) PuntoDeCarga")
        destino = input("\nEscribe el nombre del destino: ").strip()

        if not destino:
            raise ValueError("El nombre del destino no puede estar vacío.")

        navigator = WarehouseNavigator(destino)
        navigator.send_goal()

        # Girar hasta que la acción termine
        while rclpy.ok() and not navigator.goal_done:
            rclpy.spin_once(navigator, timeout_sec=0.1)

    except (FileNotFoundError, ValueError) as err:
        print(f'Error de configuración: {err}')
    except RuntimeError as err:
        print(f'Error al inicializar el nodo ROS 2: {err}')
    finally:
        try:
            navigator.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
"""
Este módulo publica la posición inicial del robot en ROS 2 para que AMCL pueda localizarse.

Classes:
    InitialPosePub: Nodo ROS 2 que publica la pose inicial en el topic /initialpose.

Functions:
    main(): Inicializa y ejecuta el nodo InitialPosePub.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePub(Node):
    """Nodo ROS 2 que publica la posición inicial del robot para que AMCL se localice.

    Espera 3 segundos tras su creación para asegurar que Nav2 esté listo,
    y publica una única vez la pose inicial en el topic /initialpose.

    Attributes:
        publisher: Publicador en el topic /initialpose.
        timer: Temporizador que dispara la publicación tras 3 segundos.
        published (bool): Indica si la pose ya ha sido publicada.

    Methods:
        publish_pose(): Construye y publica el mensaje de pose inicial.
    """

    def __init__(self):
        """Inicializa el nodo, el publicador y el temporizador.

        Raises:
            RuntimeError: Si rclpy no ha sido inicializado antes de crear el nodo.
        """
        super().__init__('initial_pose_pub')

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # Esperamos 3 s para que Nav2 esté listo antes de publicar
        self.timer = self.create_timer(3.0, self.publish_pose)
        self.published = False

    def publish_pose(self):
        """Construye y publica la pose inicial del robot en el topic /initialpose.

        Solo se ejecuta una vez aunque el temporizador la llame varias veces.
        La posición corresponde al punto de spawn definido en my_world.launch.py.

        Raises:
            Exception: Si ocurre un error al publicar el mensaje en el topic.
        """
        if self.published:
            return

        try:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()

            # Posición de spawn del robot en my_world.launch.py
            msg.pose.pose.position.x = 0.0
            msg.pose.pose.position.y = 0.0
            msg.pose.pose.position.z = 0.0

            # Orientación: mirando hacia el eje X+ (yaw = 0)
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = 0.0
            msg.pose.pose.orientation.w = 1.0

            # Covarianza estándar para AMCL
            msg.pose.covariance[0] = 0.25   # xx
            msg.pose.covariance[7] = 0.25   # yy
            msg.pose.covariance[35] = 0.07  # yaw

            self.publisher.publish(msg)
            self.get_logger().info(
                'Posición inicial publicada: x=0.0, y=0.0'
            )
            self.published = True

        except Exception as err:
            self.get_logger().error(
                f'Error al publicar la posición inicial: {err}'
            )


def main():
    """Inicializa rclpy, crea el nodo InitialPosePub y lo mantiene en ejecución.

    Raises:
        RuntimeError: Si rclpy no puede inicializarse correctamente.
    """
    try:
        rclpy.init()
        node = InitialPosePub()
        rclpy.spin(node)
    except RuntimeError as err:
        print(f'Error al inicializar el nodo: {err}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
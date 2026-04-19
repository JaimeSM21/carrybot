import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePub(Node):
    """Publica la posición inicial del robot para que AMCL se localice."""

    def __init__(self):
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
        if self.published:
            return

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


def main():
    rclpy.init()
    node = InitialPosePub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

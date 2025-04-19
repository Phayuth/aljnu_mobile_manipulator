import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__("velocity_publisher")
        self.publisher_ = self.create_publisher(Float64MultiArray, "/forward_velocity_controller/commands", 10)
        self.timer = self.create_timer(0.01, self.publish_velocity)
        self.get_logger().info("Velocity Publisher Node has been started.")

    def publish_velocity(self):

        msg = Float64MultiArray()
        msg.data = [
            1 * math.sin((2 * 3.14 / 10) * self.get_clock().now().nanoseconds * 1e-9),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_velocity()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

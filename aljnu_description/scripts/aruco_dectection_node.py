import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from aruco_detection import ARUCOBoardPose
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__("aruco_detection_node")

        self.camera_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.camera_info_callback, 10
        )
        self.camera_pub = self.create_publisher(Image, "/camera/aruco_image", 10)

        self.tf_broadcaster = None
        self.aruco_board_pose = ARUCOBoardPose()
        self.get_logger().info("Aruco Detection Node has been started.")

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_camera_to_tool0_static_transform()

    def image_callback(self, msg):
        # Process the incoming image message
        pass

    def camera_info_callback(self, msg):
        # Process the incoming camera info message
        pass

    def publish_camera_to_tool0_static_transform(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "tool0"
        static_transform.child_frame_id = "camera_link"
        static_transform.transform.translation.x = -0.0175  # m
        static_transform.transform.translation.y = -0.1  # m
        static_transform.transform.translation.z = 0.0258  # m
        static_transform.transform.rotation.x = 0.5
        static_transform.transform.rotation.y = -0.5
        static_transform.transform.rotation.z = 0.5
        static_transform.transform.rotation.w = 0.5
        self.tf_broadcaster.sendTransform(static_transform)


if __name__ == "__main__":
    rclpy.init()
    node = ArucoDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

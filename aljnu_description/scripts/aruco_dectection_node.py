import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from aruco_detection import ARUCOBoardPose
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from pytransform3d import transformations as t3d


class ArucoDetectionNode(Node):

    def __init__(self):
        super().__init__("aruco_detection_node")
        # constants
        self.frame_id = "camera_link"
        self.child_frame_id = "calib_board"
        self.camera_k = np.array(
            [
                [599.639625, 0.0, 328.841620],
                [0.0, 602.139246, 232.169169],
                [0.0, 0.0, 1.0],
            ]
        )

        self.camera_d = np.array(
            [0.143990, -0.280626, 0.002779, -0.000829, 0.000000]
        )

        # ros
        self.camera_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera_info",
            self.camera_info_callback,
            10,
        )
        self.camera_pub = self.create_publisher(
            Image,
            "/camera/aruco_image",
            10,
        )

        # class
        self.aruco_board_pose = ARUCOBoardPose()
        self.bridge = CvBridge()
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.save_info = False

    def image_callback(self, msg):
        if not self.save_info:
            self.get_logger().warn("Waiting for camera info...")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            pos, R = self.aruco_board_pose.run(
                self.camera_k,
                self.camera_d,
                cv_image,
            )
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = pos
            pq = t3d.pq_from_transform(T)
            self.pub_tf(pq[:3], pq[3:7])

            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.camera_pub.publish(ros_image)

        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")

    def camera_info_callback(self, msg):
        self.camera_k = np.array(msg.k).reshape(3, 3)
        self.camera_d = np.array(msg.d)
        self.get_logger().info(f"Camera K: {self.camera_k}")
        self.get_logger().info(f"Camera D: {self.camera_d}")
        self.camera_info_sub.destroy()
        self.save_info = True

    def pub_tf(self, position, orientation):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = self.child_frame_id
        tf.transform.translation.x = position[0]
        tf.transform.translation.y = position[1]
        tf.transform.translation.z = position[2]
        tf.transform.rotation.w = orientation[0]
        tf.transform.rotation.x = orientation[1]
        tf.transform.rotation.y = orientation[2]
        tf.transform.rotation.z = orientation[3]
        self.tf_broadcaster.sendTransform(tf)


if __name__ == "__main__":
    rclpy.init()
    node = ArucoDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

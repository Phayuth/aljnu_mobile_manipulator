import rclpy
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3, PoseStamped
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path
import numpy as np


class CubicBezierBoxMarker(Node):
    def __init__(self):
        super().__init__("box_marker_controller")

        self.server = InteractiveMarkerServer(self, "marker_server")

        # Control points
        self.P0 = np.array([0.0, 0.0])
        self.P1 = np.array([1.0, 2.0])
        self.P2 = np.array([3.0, 3.0])
        self.P3 = np.array([4.0, 0.0])

        self.create_box_marker("start", self.P0[0], self.P0[1], 0.0)
        self.create_box_marker("c1", self.P1[0], self.P1[1], 0.0)
        self.create_box_marker("c2", self.P2[0], self.P2[1], 0.0)
        self.create_box_marker("end", self.P3[0], self.P3[1], 0.0)

        self.server.applyChanges()

        self.pathpuber = self.create_publisher(Path, "path", 10)
        self.timers_ = self.create_timer(0.5, self.create_path_msg)

    def process_feedback(self, feedback):
        p = feedback.pose.position
        if feedback.marker_name == "start":
            self.P0[0] = p.x
            self.P0[1] = p.y
        if feedback.marker_name == "c1":
            self.P1[0] = p.x
            self.P1[1] = p.y
        if feedback.marker_name == "c2":
            self.P2[0] = p.x
            self.P2[1] = p.y
        if feedback.marker_name == "end":
            self.P3[0] = p.x
            self.P3[1] = p.y

    def create_box_marker(self, name, x, y, z):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = name
        int_marker.description = name
        int_marker.pose.position = Point(x=x, y=y, z=z)

        box = Marker()
        box.type = Marker.SPHERE
        box.scale = Vector3(x=0.2, y=0.2, z=0.2)
        box.color = ColorRGBA(r=0.0, g=0.5, b=0.5, a=1.0)

        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box)
        int_marker.controls.append(box_control)

        move_plane_control = InteractiveMarkerControl()
        move_plane_control.orientation.w = 1.0
        move_plane_control.orientation.x = 0.0
        move_plane_control.orientation.y = 1.0
        move_plane_control.orientation.z = 0.0
        move_plane_control.name = "move_plane"
        move_plane_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker.controls.append(move_plane_control)

        self.server.insert(int_marker, feedback_callback=self.process_feedback)

    def generate_bezier(self):
        t = np.linspace(0, 1, 100)
        B_t = np.outer((1 - t) ** 3, self.P0) + np.outer(3 * t * (1 - t) ** 2, self.P1) + np.outer(3 * t**2 * (1 - t), self.P2) + np.outer(t**3, self.P3)
        return B_t

    def create_path_msg(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "base_link"

        B_t = self.generate_bezier()
        for i in range(100):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position = Point(x=B_t[i, 0], y=B_t[i, 1], z=0.0)
            path.poses.append(pose)

        self.pathpuber.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = CubicBezierBoxMarker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.server.clear()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

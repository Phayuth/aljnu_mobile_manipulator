#include "aljnu_path/bezier.h"

Bezier::Bezier() : Node("bezier_path") {
    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "marker", this);
    path_publisher_ = this->create_publisher<PathMsg>("path", 10);

    p0 = {0.0, 0.0};
    p1 = {0.0, 0.0};
    p2 = {0.0, 0.0};
    p3 = {0.0, 0.0};

    create_marker("start", 0.0, 0.0, 0.0);
    create_marker("c1", 0.0, 0.0, 0.0);
    create_marker("c2", 0.0, 0.0, 0.0);
    create_marker("end", 0.0, 0.0, 0.0);

    server_->applyChanges();
}

Bezier::~Bezier() {
}

void Bezier::compute_pose(double t, double &x, double &y) {
    // control points
    x = (1 - t) * ((1 - t) * ((1 - t) * p0[0] + t * p1[0]) +
                   t * ((1 - t) * p1[0] + t * p2[0])) +
        t * ((1 - t) * ((1 - t) * p1[0] + t * p2[0]) +
             t * ((1 - t) * p2[0] + t * p3[0]));
    y = (1 - t) * ((1 - t) * ((1 - t) * p0[1] + t * p1[1]) +
                   t * ((1 - t) * p1[1] + t * p2[1])) +
        t * ((1 - t) * ((1 - t) * p1[1] + t * p2[1]) +
             t * ((1 - t) * p2[1] + t * p3[1]));
}

void Bezier::compute_velo(double t) {
    (void)t;
}

void Bezier::compute_accl(double t) {
    (void)t;
}

void Bezier::create_marker(const std::string &name, double x, double y, double z) {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    int_marker.name = name;
    int_marker.description = name;
    int_marker.pose.position.x = x;
    int_marker.pose.position.y = y;
    int_marker.pose.position.z = z;

    visualization_msgs::msg::Marker box;
    box.type = visualization_msgs::msg::Marker::SPHERE;
    box.scale.x = 0.2;
    box.scale.y = 0.2;
    box.scale.z = 0.2;
    box.color.a = 1.0;
    box.color.r = 0.0;
    box.color.g = 0.5;
    box.color.b = 0.5;

    visualization_msgs::msg::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(box);
    int_marker.controls.push_back(box_control);

    visualization_msgs::msg::InteractiveMarkerControl move_plane_control;
    move_plane_control.orientation.w = 1.0;
    move_plane_control.orientation.x = 0.0;
    move_plane_control.orientation.y = 1.0;
    move_plane_control.orientation.z = 0.0;
    move_plane_control.name = "move_plane";
    move_plane_control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(move_plane_control);

    server_->insert(
        int_marker,
        std::bind(&Bezier::process_feedback, this, std::placeholders::_1));
}

void Bezier::process_feedback(const IntMarkerFBMsg::ConstSharedPtr &feedback) {
    if (feedback->marker_name == "start") {
        p0[0] = feedback->pose.position.x;
        p0[1] = feedback->pose.position.y;
    }
    if (feedback->marker_name == "c1") {
        p1[0] = feedback->pose.position.x;
        p1[1] = feedback->pose.position.y;
    }
    if (feedback->marker_name == "c2") {
        p2[0] = feedback->pose.position.x;
        p2[1] = feedback->pose.position.y;
    }
    if (feedback->marker_name == "end") {
        p3[0] = feedback->pose.position.x;
        p3[1] = feedback->pose.position.y;
    }
    publish_path();
}

void Bezier::publish_path() {
    PathMsg path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "base_link";
    double x;
    double y;
    for (size_t i = 0; i < 100; i++) {
        PoseStampedMsg pose;
        pose.header = path.header;
        compute_pose(i / 100.0, x, y);
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        path.poses.push_back(pose);
    }
    path_publisher_->publish(path);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Bezier>());
    rclcpp::shutdown();
    return 0;
}
#ifndef ALJNU_BEZIER_PATH_H
#define ALJNU_BEZIER_PATH_H

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using PathMsg = nav_msgs::msg::Path;
using IntMarkerFBMsg = visualization_msgs::msg::InteractiveMarkerFeedback;

class Bezier : public rclcpp::Node {
    private:
        std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
        rclcpp::Publisher<PathMsg>::SharedPtr path_publisher_;

        std::array<double, 2> p0;
        std::array<double, 2> p1;
        std::array<double, 2> p2;
        std::array<double, 2> p3;

        void process_feedback(const IntMarkerFBMsg::ConstSharedPtr &feedback);
        void create_marker(const std::string &name, double x, double y, double z);
        void publish_path();

    public:
        Bezier();
        ~Bezier();

        void compute_pose(double t, double &x, double &y);
        void compute_velo(double t);
        void compute_accl(double t);
};

#endif
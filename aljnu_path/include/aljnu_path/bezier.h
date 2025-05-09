#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

using TrajectoryMsg = trajectory_msgs::msg::JointTrajectory;
using TrajectoryPointMsg = trajectory_msgs::msg::JointTrajectoryPoint;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using PathMsg = nav_msgs::msg::Path;

class Bezier : public rclcpp::Node {
    private:
        rclcpp::Publisher<TrajectoryMsg>::SharedPtr pub_;
        rclcpp::Subscription<PoseStampedMsg>::SharedPtr sub_;


        void goal_callback(const PoseStampedMsg::ConstPtr &msg);

    public:
        Bezier();
        void compute_pose(double t);
        void compute_velo(double t);
        void compute_accl(double t);
};

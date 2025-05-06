#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using TrajectoryMsg = trajectory_msgs::msg::JointTrajectory;
using TrajectoryPointMsg = trajectory_msgs::msg::JointTrajectoryPoint;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;

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

#ifndef ALJNU_TRAJECTORY_CONTROLLER
#define ALJNU_TRAJECTORY_CONTROLLER

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/rclcpp.hpp>

class TrajectoryBacksteppingController : public rclcpp::Node {
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twtpub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomsub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalsub_;

        // physical limit
        double vlimit;
        double wlimit;

        // kinematic controller constants for tuning
        double k1;
        double k2;
        double k3;

        // dynamic controller constants for tuning
        double ka;
        double kb;

        double dTol;

        // reference pose
        double xref;
        double yref;
        double yawref;
        double vref;
        double wref;

        // control
        double vc;
        double wc;

        // topic name
        std::string twist_topic;
        std::string odometry_topic;
        std::string goal_topic;

    public:
        TrajectoryBacksteppingController();
        double compute_yaw(double qw, double qx, double qy, double qz);
        void compute_control(double x, double y, double yaw, double &vc, double &wc);
        void get_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void velocity_limit(double &vc, double &wc);
        void loop(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif
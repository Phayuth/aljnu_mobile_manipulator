#ifndef ALJNU_POSITION_CONTROLLER_H
#define ALJNU_POSITION_CONTROLLER_H

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class PositionController : public rclcpp::Node {
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twtpub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomsub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalsub_;

        // physical limit
        double vlimit;
        double wlimit;

        // reference pose
        double xref;
        double yref;
        double yawref;
        double phiref;

        // control velocity
        double vc;
        double wc;

        // control tune
        double k1;
        double k2;
        double dTol;

        bool isReady = true;

        // topic name
        std::string twist_topic;
        std::string odometry_topic;
        std::string goal_topic;

    public:
        PositionController();
        double compute_yaw(double qw, double qx, double qy, double qz);
        void compute_control(double x, double y, double yaw, double &vc,
                             double &wc);
        void get_goal(geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void velocity_limit(double &vc, double &wc);
        void loop(nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif
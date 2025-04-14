#include <memory>

#include "aljnu_controllers/backstepping_controller.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>

TrajectoryBacksteppingController::TrajectoryBacksteppingController() : Node("backstepping_controller") {
    this->declare_parameter<double>("limit.v", 0.0);
    this->declare_parameter<double>("limit.w", 0.0);
    this->declare_parameter<double>("controller_gain.k1", 0.0);
    this->declare_parameter<double>("controller_gain.k2", 0.0);
    this->declare_parameter<double>("controller_gain.k3", 0.0);
    this->declare_parameter<double>("controller_gain.ka", 0.0);
    this->declare_parameter<double>("controller_gain.kb", 0.0);
    this->declare_parameter<double>("controller_gain.dTol", 0.0);
    this->declare_parameter<std::string>("topic.pub.twist", "/cmd_vel");
    this->declare_parameter<std::string>("topic.sub.odometry", "/odom");
    this->declare_parameter<std::string>("topic.sub.goal", "/goal_pose");

    this->get_parameter("limit.v", vlimit);
    this->get_parameter("limit.w", wlimit);
    this->get_parameter("controller_gain.k1", k1);
    this->get_parameter("controller_gain.k2", k2);
    this->get_parameter("controller_gain.k3", k3);
    this->get_parameter("controller_gain.ka", ka);
    this->get_parameter("controller_gain.kb", kb);
    this->get_parameter("controller_gain.dTol", dTol);
    this->get_parameter("topic.pub.twist", twist_topic);
    this->get_parameter("topic.sub.odometry", odometry_topic);
    this->get_parameter("topic.sub.goal", goal_topic);

    using std::placeholders::_1;
    twtpub_ = this->create_publisher<geometry_msgs::msg::Twist>(twist_topic, 10);
    odomsub_ = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic, 10, std::bind(&TrajectoryBacksteppingController::loop, this, _1));
    goalsub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(goal_topic, 10, std::bind(&TrajectoryBacksteppingController::get_goal, this, _1));

    RCLCPP_INFO(this->get_logger(), "Backstepping Controller Node is initialized!");
}

void TrajectoryBacksteppingController::get_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    xref = msg->pose.position.x;
    yref = msg->pose.position.y;
    yawref = compute_yaw(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    vref = 0.0;
    wref = 0.0;
}

double TrajectoryBacksteppingController::compute_yaw(double qw, double qx, double qy, double qz) {
    return atan2(2.0f * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz);
}

void TrajectoryBacksteppingController::compute_control(double x, double y, double yaw, double &vc, double &wc) {
    double ex = xref - x;
    double ey = yref - y;
    double eyaw = yawref - yaw;

    double qex = cos(yaw) * ex + sin(yaw) * ey;
    double qey = -sin(yaw) * ex + cos(yaw) * ey;
    double qeyaw = eyaw;

    vc = vref * cos(qeyaw) + k1 * qex;
    wc = wref + k2 * vref * qey + k3 * sin(qeyaw);

    if (sqrt(qex * qex + qey + qey) < dTol) {
        vc = 0.0;
        wc = 0.0;
    }
}

void TrajectoryBacksteppingController::velocity_limit(double &vc, double &wc) {
    if (vc > vlimit) {
        vc = vlimit;
    }
    if (vc < -vlimit) {
        vc = -vlimit;
    }
    if (wc > wlimit) {
        wc = wlimit;
    }
    if (wc < -wlimit) {
        wc = -wlimit;
    }
}

void TrajectoryBacksteppingController::loop(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double yaw = compute_yaw(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    compute_control(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw, vc, wc);

    velocity_limit(vc, wc);

    geometry_msgs::msg::Twist tmsg;
    tmsg.linear.x = vc;
    tmsg.angular.z = wc;
    twtpub_->publish(tmsg);
}

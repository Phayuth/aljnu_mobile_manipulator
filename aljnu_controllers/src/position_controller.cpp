#include <memory>

#include "aljnu_controllers/position_controller.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/parameter.hpp"

PositionController::PositionController() : rclcpp::Node("position_controller") {
    this->declare_parameter<double>("limit.v", 0.0);
    this->declare_parameter<double>("limit.w", 0.0);
    this->declare_parameter<double>("controller_gain.k1", 0.0);
    this->declare_parameter<double>("controller_gain.k2", 0.0);
    this->declare_parameter<double>("controller_gain.dTol", 0.0);
    this->declare_parameter<std::string>("topic.pub.twist", "/cmd_vel");
    this->declare_parameter<std::string>("topic.sub.odometry", "/odom");
    this->declare_parameter<std::string>("topic.sub.goal", "/goal_pose");

    this->get_parameter("limit.v", vlimit);
    this->get_parameter("limit.w", wlimit);
    this->get_parameter("controller_gain.k1", k1);
    this->get_parameter("controller_gain.k2", k2);
    this->get_parameter("controller_gain.dTol", dTol);
    this->get_parameter("topic.pub.twist", twist_topic);
    this->get_parameter("topic.sub.odometry", odometry_topic);
    this->get_parameter("topic.sub.goal", goal_topic);

    using std::placeholders::_1;
    twtpub_ = this->create_publisher<geometry_msgs::msg::Twist>(twist_topic, 10);
    odomsub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic, 10, std::bind(&PositionController::loop, this, _1));
    goalsub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic, 10, std::bind(&PositionController::get_goal, this, _1));

    RCLCPP_INFO(this->get_logger(), "Position Controller Node is initialized...!");
}

double PositionController::compute_yaw(double qw, double qx, double qy,
                                       double qz) {
    return atan2(2.0f * (qw * qz + qx * qy),
                 qw * qw + qx * qx - qy * qy - qz * qz);
}

void PositionController::compute_control(double x, double y, double yaw,
                                         double &vc, double &wc) {
    phiref = atan2(yref - y, xref - x);
    double exy = sqrt(pow(xref - x, 2) + pow(yref - y, 2));
    vc = k1 * exy;
    wc = k2 * (phiref - yaw);

    if (exy < dTol) {
        vc = 0.0;
        wc = 0.0;
    }
}

void PositionController::get_goal(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    xref = msg->pose.position.x;
    yref = msg->pose.position.y;
    yawref = compute_yaw(msg->pose.orientation.w,
                         msg->pose.orientation.x,
                         msg->pose.orientation.y,
                         msg->pose.orientation.z);
}

void PositionController::velocity_limit(double &vc, double &wc) {
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
    RCLCPP_INFO(this->get_logger(),
                "xref:%f, yref:%f, yawref:%f, vc:%f, wc:%f",
                xref,
                yref,
                yawref,
                vc,
                wc);
}

void PositionController::loop(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double yaw = compute_yaw(msg->pose.pose.orientation.w,
                             msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y,
                             msg->pose.pose.orientation.z);

    compute_control(
        msg->pose.pose.position.x, msg->pose.pose.position.y, yaw, vc, wc);

    velocity_limit(vc, wc);

    geometry_msgs::msg::Twist tmsg;
    tmsg.linear.x = vc;
    tmsg.angular.z = wc;
    twtpub_->publish(tmsg);
}
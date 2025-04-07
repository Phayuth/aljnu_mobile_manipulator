#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen.h>

class TrajectoryBacksteppingController : public rclcpp::Node {
    private:
        // physical limit
        double const vlimit = 1.0;
        double const wlimit = 1.0;

        // kinematic controller constants for tuning
        double k1 = 10.0;
        double k2 = 5.0;
        double k3 = 4.0;

        // dynamic controller constants for tuning
        double ka = 100.0;
        double kb = 3000.0;

    public:
        TrajectoryBacksteppingController();
        void generate_bezier_trajectory(double t);
        double compute_yaw(double qw, double qx, double qy, double qz);
        void compute_control(double x, double y, double yaw, double &vc, double &wc);
        void velocity_limit(double &vc, double &wc);
        void loop(const nav_msgs::msg::Odometry::SharedPtr msg);
};

TrajectoryBacksteppingController::TrajectoryBacksteppingController() : Node("control") {
}

void TrajectoryBacksteppingController::generate_bezier_trajectory(double t) {
    // control points
    double p0[2] = {0.0, 0.0};
    double p1[2] = {1.0, 1.0};
    double p2[2] = {2.0, 1.0};
    double p3[2] = {3.0, 0.0};

    double x = (1 - t) * ((1 - t) * ((1 - t) * p0[0] + t * p1[0]) + t * ((1 - t) * p1[0] + t * p2[0])) + t * ((1 - t) * ((1 - t) * p1[0] + t * p2[0]) + t * ((1 - t) * p2[0] + t * p3[0]));
    double y = (1 - t) * ((1 - t) * ((1 - t) * p0[1] + t * p1[1]) + t * ((1 - t) * p1[1] + t * p2[1])) + t * ((1 - t) * ((1 - t) * p1[1] + t * p2[1]) + t * ((1 - t) * p2[1] + t * p3[1]));
}

double TrajectoryBacksteppingController::compute_yaw(double qw, double qx, double qy, double qz) {
    return atan2(2.0f * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz);
}

void TrajectoryBacksteppingController::compute_control(double x, double y, double yaw, double &vc, double &wc) {
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    T(0, 0) = cos(yaw);
    T(0, 1) = sin(yaw);
    T(1, 0) = -sin(yaw);
    T(1, 1) = cos(yaw);
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
}

int main(int argc, char const *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryBacksteppingController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
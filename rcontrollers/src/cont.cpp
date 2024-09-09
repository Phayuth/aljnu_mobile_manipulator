#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class Controllers : public rclcpp::Node {
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twtpub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomsub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalsub_;

        // physical limit
        double const vlimit = 1.0;
        double const wlimit = 1.0;

        // reference pose
        double xref;
        double yref;
        double yawref;
        double phiref;

        // control velocity
        double vc;
        double wc;

        // control tune
        double const k1 = 0.2;
        double const k2 = 1.2;
        double const dTol = 0.1;

        bool isReady = true;

    public:
        Controllers();
        double compute_yaw(double qw, double qx, double qy, double qz);
        void compute_control(double x, double y, double yaw, double &vc, double &wc);
        void get_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void velocity_limit(double &vc, double &wc);
        void loop(const nav_msgs::msg::Odometry::SharedPtr msg);
};

Controllers::Controllers() : Node("control") {
    twtpub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    using std::placeholders::_1;
    odomsub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&Controllers::loop, this, _1));
    goalsub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&Controllers::get_goal, this, _1));
}

double Controllers::compute_yaw(double qw, double qx, double qy, double qz) {
    return atan2(2.0f * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz);
}

void Controllers::compute_control(double x, double y, double yaw, double &vc, double &wc) {
    phiref = atan2(yref - y, xref - x);
    double exy = sqrt(pow(xref - x, 2) + pow(yref - y, 2));
    vc = k1 * exy;
    wc = k2 * (phiref - yaw);

    if (exy < dTol) {
        vc = 0.0;
        wc = 0.0;
    }
}

void Controllers::get_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    xref = msg->pose.position.x;
    yref = msg->pose.position.y;
    yawref = Controllers::compute_yaw(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
}

void Controllers::velocity_limit(double &vc, double &wc) {
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
    RCLCPP_INFO(this->get_logger(), "xref:%f, yref:%f, yawref:%f, vc:%f, wc:%f", xref, yref, yawref, vc, wc);
}

void Controllers::loop(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double yaw = Controllers::compute_yaw(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    // compute control velocity
    Controllers::compute_control(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw, vc, wc);

    // limit
    Controllers::velocity_limit(vc, wc);

    // compose and send
    geometry_msgs::msg::Twist tmsg;
    tmsg.linear.x = vc;
    tmsg.angular.z = wc;
    twtpub_->publish(tmsg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controllers>());
    rclcpp::shutdown();
    return 0;
}
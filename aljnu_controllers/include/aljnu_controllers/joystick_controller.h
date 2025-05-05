#ifndef ALJNU_JOYSTICK_CONTROLLER
#define ALJNU_JOYSTICK_CONTROLLER

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <atomic>
#include <linux/joystick.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <thread>

using TwistMsg = geometry_msgs::msg::Twist;
using LifecycleCallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class JoystickController : public rclcpp_lifecycle::LifecycleNode {
    private:
        double scale_linear_vel_;
        double scale_angular_vel_;
        double scale_linear_deadzone_;
        double scale_angular_deadzone_;
        std::string cmd_topic_;

        int joystick_fd_;
        std::string joystick_device_;
        std::atomic<bool> running_;
        std::thread joystick_thread_;

        rclcpp::Publisher<TwistMsg>::SharedPtr twist_publisher_;
        TwistMsg twist_msg_;

        void joystick_read();
        void zeros_out_twist();
        void try_open_controller();

    public:
        JoystickController();
        ~JoystickController();

        LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &prev_state);
        LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &prev_state);
        LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &prev_state);
        LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &prev_state);
        LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &prev_state);
        LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &prev_state);
};

#endif
#include "aljnu_controllers/joystick_controller.h"
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

using namespace std::chrono_literals;

JoystickController::JoystickController() : LifecycleNode("joystick_controller") {
    RCLCPP_INFO(this->get_logger(), "In Constuctor");

    this->declare_parameter("joystick_device", "/dev/input/js0");
    this->declare_parameter("scale_linear_vel", 1.0);
    this->declare_parameter("scale_angular_vel", 1.0);
    this->declare_parameter("scale_linear_deadzone", 0.05);
    this->declare_parameter("scale_angular_deadzone", 0.05);
    this->declare_parameter("cmd_topic_name", "/cmd_vel");
}

void JoystickController::try_open_controller() {
    joystick_device_ = this->get_parameter("joystick_device").as_string();
    joystick_fd_ = open(joystick_device_.c_str(), O_RDONLY | O_NONBLOCK);
}

void JoystickController::zeros_out_twist() {
    twist_msg_.linear.x = 0.0;
    twist_msg_.angular.z = 0.0;
    twist_publisher_->publish(twist_msg_);
}

LifecycleCallbackReturn
JoystickController::on_configure(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In Configure");
    (void)prev_state;

    // ROS
    scale_linear_vel_ = this->get_parameter("scale_linear_vel").as_double();
    scale_angular_vel_ = this->get_parameter("scale_angular_vel").as_double();
    scale_linear_deadzone_ =
        this->get_parameter("scale_linear_deadzone").as_double();
    scale_angular_deadzone_ =
        this->get_parameter("scale_angular_deadzone").as_double();
    cmd_topic_ = this->get_parameter("cmd_topic_name").as_string();

    twist_publisher_ = this->create_publisher<TwistMsg>(cmd_topic_, 10);
    running_ = false;

    // Open  Hardware joystick device
    try_open_controller();
    if (joystick_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to established connection to joystick device: %s",
                     joystick_device_.c_str());
        return LifecycleCallbackReturn::FAILURE;
    }
    RCLCPP_INFO(this->get_logger(),
                "Successfully to established connection to joystick device.");
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn
JoystickController::on_cleanup(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In Cleanup");
    (void)prev_state;

    zeros_out_twist();

    twist_publisher_.reset();
    if (joystick_fd_ >= 0) {
        close(joystick_fd_);
    }
    RCLCPP_INFO(this->get_logger(), "Joystick is closed!.");

    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn
JoystickController::on_activate(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In activate");

    // Start joystick reading thread
    try {
        running_ = true;
        joystick_thread_ = std::thread(&JoystickController::joystick_read, this);
    } catch (const std::exception &e) {
        RCLCPP_INFO(this->get_logger(), e.what());
        return LifecycleCallbackReturn::FAILURE;
    }
    rclcpp_lifecycle::LifecycleNode::on_activate(prev_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn
JoystickController::on_deactivate(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In deactivate");
    running_ = false;

    zeros_out_twist();
    if (joystick_thread_.joinable()) {
        joystick_thread_.join();
    }

    rclcpp_lifecycle::LifecycleNode::on_deactivate(prev_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn
JoystickController::on_shutdown(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In shutdown");
    (void)prev_state;

    zeros_out_twist();
    twist_publisher_.reset();
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn
JoystickController::on_error(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In error");
    (void)prev_state;

    // ensure if there is error, the robot doesn't move.
    running_ = false;
    zeros_out_twist();

    // do some checking later
    return LifecycleCallbackReturn::FAILURE;
}

JoystickController::~JoystickController() {
}

void JoystickController::joystick_read() {
    struct js_event event;
    float linear = 0.0, angular = 0.0;

    while (running_) {
        ssize_t bytes = read(joystick_fd_, &event, sizeof(event));

        if (bytes < 0) {
            if (errno != EAGAIN) { // Ignore non-blocking errors
                RCLCPP_ERROR(this->get_logger(),
                             "Error reading joystick: %s",
                             strerror(errno));
                running_ = false;
                zeros_out_twist();
                break;
            }
        }

        if (bytes == sizeof(event)) {
            event.type &= ~JS_EVENT_INIT; // Ignore initialization events
            if (event.type == JS_EVENT_AXIS) {
                if (event.number == 1) {
                    linear = -scale_linear_vel_ * (event.value / 32767.0);
                    if ((-scale_linear_deadzone_ <= linear) &&
                        (linear <= scale_linear_deadzone_)) {
                        linear = 0.0;
                    }
                } else if (event.number == 3) {
                    angular = -scale_angular_vel_ * (event.value / 32767.0);
                    if ((-scale_angular_deadzone_ <= angular) &&
                        (angular <= scale_angular_deadzone_)) {
                        angular = 0.0;
                    }
                }
            }

            twist_msg_.linear.x = linear;
            twist_msg_.angular.z = angular;
            twist_publisher_->publish(twist_msg_);
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(10)); // Avoid busy looping
    }
}

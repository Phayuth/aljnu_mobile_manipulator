#ifndef ALJNU_ADMITTANCE_CONTROLLER
#define ALJNu_ADMITTANCE_CONTROLLER

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// #include <aljnu_controllers/kdlsolver.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>

using FloatMsg = std_msgs::msg::Float64MultiArray;
using WrenchMsg = geometry_msgs::msg::WrenchStamped;
using LifecycleCallbackReturn = rclcpp_lifecycle::LifecycleNode::CallbackReturn;

class AdmittanceController : public rclcpp_lifecycle::LifecycleNode {
    private:
        std::string ft_sensor_tf_;
        std::string base_tf_;
        std::string joint_command_topic_;
        std::string ft_wrench_topic_;

        std::vector<bool> applied_axis;
        std::vector<double> mass;
        std::vector<double> damping;
        std::vector<double> stiffness;

        FloatMsg joint_value;

        rclcpp::Publisher<FloatMsg>::SharedPtr pub_;
        rclcpp::Subscription<WrenchMsg>::SharedPtr sub_;

        void wrench_callback(const WrenchMsg::SharedPtr msg);

    public:
        AdmittanceController();
        ~AdmittanceController();

        LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &prev_state);
        LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &prev_state);
        LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &prev_state);
        LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &prev_state);
        LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &prev_state);
        LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &prev_state);
};

#endif
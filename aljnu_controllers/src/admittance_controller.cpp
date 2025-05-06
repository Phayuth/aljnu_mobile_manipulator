#include "aljnu_controllers/admittance_controller.h"

AdmittanceController::AdmittanceController() : LifecycleNode("admittance_controller") {
    RCLCPP_INFO(get_logger(), "In Constructor");

    this->declare_parameter<std::string>("ft_sensor_tf", "/ft_sensor");
    this->declare_parameter<std::string>("base_tf", "/base_link");
    this->declare_parameter<std::string>("joint_command_topic", "/command");
    this->declare_parameter<std::string>("ft_wrench_topic", "/wrench");

    this->declare_parameter<std::vector<bool>>("applied_axis", {false, false, false, false, false, false});
    this->declare_parameter<std::vector<double>>("mass", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("damping", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("stiffness", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

AdmittanceController::~AdmittanceController() {
}

void AdmittanceController::wrench_callback(const WrenchMsg::SharedPtr msg) {
    // TODO: DO your admittance here by converting force to tip velocity.
    // using wrench data here
    msg->wrench.force.x;
    msg->wrench.force.y;
    msg->wrench.force.z;
    msg->wrench.torque.x;
    msg->wrench.torque.y;
    msg->wrench.torque.z;

    // Wrench -> admittance -> Twist (tip)
    // Careful when when transform data in frame;
    // geometry_msgs::msg::WrenchStamped wrench_in_base;
    // tf2::doTransform(wrench_in_tip, wrench_in_base, transform_tip_to_base);

    // make twist data
    KDL::Vector lin_vel(0.1, 0.0, 0.0);
    KDL::Vector rot_vel(0.1, 0.0, 0.0);
    KDL::Twist v_tip(lin_vel, rot_vel);

    // use kdlsolver to compute q_dot from twist
    KDL::JntArray q_init(6);
    q_init.data << 0., 0., 0., 0., 0., 0.;

    KDL::JntArray q_dot(6);
    q_dot.data << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    // kdlsolver.ik_vel(q_init, v_tip, q_dot);

    // publish q_dot on topic
    joint_value.data.resize(6);
    joint_value.data[0] = q_dot.data[0];
    joint_value.data[1] = q_dot.data[1];
    joint_value.data[2] = q_dot.data[2];
    joint_value.data[3] = q_dot.data[3];
    joint_value.data[4] = q_dot.data[4];
    joint_value.data[5] = q_dot.data[5];
    pub_->publish(joint_value);
}

LifecycleCallbackReturn AdmittanceController::on_configure(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In configure");
    (void)prev_state;

    ft_sensor_tf_ = this->get_parameter("ft_sensor_tf").as_string();
    base_tf_ = this->get_parameter("base_tf").as_string();
    joint_command_topic_ = this->get_parameter("joint_command_topic").as_string();
    ft_wrench_topic_ = this->get_parameter("ft_wrench_topic").as_string();

    applied_axis = this->get_parameter("applied_axis").as_bool_array();
    mass = this->get_parameter("mass").as_double_array();
    damping = this->get_parameter("damping").as_double_array();
    stiffness = this->get_parameter("stiffness").as_double_array();

    pub_ = this->create_publisher<FloatMsg>(joint_command_topic_, 10);
    sub_ = this->create_subscription<WrenchMsg>(ft_wrench_topic_, 10, std::bind(&AdmittanceController::wrench_callback, this, std::placeholders::_1));
    if (false) {
        return LifecycleCallbackReturn::FAILURE;
    }
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn AdmittanceController::on_activate(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In activate");
    rclcpp_lifecycle::LifecycleNode::on_activate(prev_state); // handle pub sub activate
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn AdmittanceController::on_cleanup(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In cleanup");
    (void)prev_state;
    sub_.reset();
    pub_.reset();
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn AdmittanceController::on_deactivate(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In deactivate");
    rclcpp_lifecycle::LifecycleNode::on_deactivate(prev_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn AdmittanceController::on_shutdown(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In shutdown");
    (void)prev_state;
    sub_.reset();
    pub_.reset();
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn AdmittanceController::on_error(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In error");
    (void)prev_state;
    return LifecycleCallbackReturn::SUCCESS;
}
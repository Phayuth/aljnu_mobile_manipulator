#include "aljnu_controllers/admittance_controller.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

AdmittanceController::AdmittanceController()
    : LifecycleNode("admittance_controller") {
    RCLCPP_INFO(get_logger(), "In Constructor");

    this->declare_parameter<std::string>("ft_sensor_tf", "/ur5e_tool0");
    this->declare_parameter<std::string>("base_tf", "/ur5e_base_link");
    this->declare_parameter<std::string>("joint_command_topic", "/command");
    this->declare_parameter<std::string>("ft_wrench_topic", "/wrench");
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states");

    this->declare_parameter<std::vector<bool>>(
        "applied_axis", {false, false, false, false, false, false});
    this->declare_parameter<std::vector<double>>("mass",
                                                 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("damping",
                                                 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("stiffness",
                                                 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

AdmittanceController::~AdmittanceController() {
}

void AdmittanceController::get_q_qdot_current(const JStateMsg::SharedPtr msg) {
    /* carefull with joint order
    - shoulder_lift_joint
    - elbow_joint
    - wrist_1_joint
    - wrist_2_joint
    - wrist_3_joint
    - shoulder_pan_joint
    position
    velocity
    effort
    */
    q_current.data << msg->position[5], msg->position[0], msg->position[1],
        msg->position[2], msg->position[3], msg->position[4];
    qdot_current.data << msg->velocity[5], msg->velocity[0], msg->velocity[1],
        msg->velocity[2], msg->velocity[3], msg->velocity[4];

    if (first_joint) {
        kdlsolver->fk_vel(q_current, qdot_current, H_Hdot_tool0InBase_current);
        H_tool0InBase_0 = H_Hdot_tool0InBase_current.GetFrame();
        first_joint = false;
    }
    // RCLCPP_INFO(get_logger(), "I got q.");
}

void AdmittanceController::get_wrench_current(const WrenchMsg::SharedPtr msg) {
    // get wrench data in ft_frame
    // tool0wrench = KDL::Wrench(
    //     KDL::Vector(msg->wrench.force.x, msg->wrench.force.y,
    //     msg->wrench.force.z), KDL::Vector(
    //         msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z));

    tool0wrench = KDL::Wrench(
        KDL::Vector(alpha * msg->wrench.force.x + (1 - alpha) * oldwrench.force[0],
                    alpha * msg->wrench.force.y + (1 - alpha) * oldwrench.force[1],
                    alpha * msg->wrench.force.z +
                        (1 - alpha) * oldwrench.force[2]),
        KDL::Vector(
            alpha * msg->wrench.torque.x + (1 - alpha) * oldwrench.torque[0],
            alpha * msg->wrench.torque.y + (1 - alpha) * oldwrench.torque[1],
            alpha * msg->wrench.torque.z + (1 - alpha) * oldwrench.torque[2]));
    // tool0wrench =
    //     KDL::Wrench(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));

    oldwrench = tool0wrench;
    // RCLCPP_INFO(get_logger(), "I got wrench.");
}

void AdmittanceController::compute_admittance() {
    // compute tool0 pose and twist in base
    kdlsolver->fk_vel(q_current, qdot_current, H_Hdot_tool0InBase_current);

    // convert wrench tool to wrench base
    H_tool0InBase_current = H_Hdot_tool0InBase_current.GetFrame();
    kdlsolver->transform_wrench(H_tool0InBase_current, tool0wrench, basewrench);

    // Wrench -> admittance -> Twist (tip)
    Hdot_tool0InBase_current = H_Hdot_tool0InBase_current.GetTwist();
    Hdot_tool0InBase_desired = KDL::Twist(
        KDL::Vector(
            (1.0 * dt / mass[0]) *
                (basewrench.force[0] - damping[0] * Hdot_tool0InBase_current[0] -
                 stiffness[0] *
                     (H_tool0InBase_current.p[0] - H_tool0InBase_0.p[0])),
            (1.0 * dt / mass[1]) *
                (basewrench.force[1] - damping[1] * Hdot_tool0InBase_current[1] -
                 stiffness[1] *
                     (H_tool0InBase_current.p[1] - H_tool0InBase_0.p[1])),
            (1.0 * dt / mass[2]) *
                (basewrench.force[2] - damping[2] * Hdot_tool0InBase_current[2] -
                 stiffness[2] *
                     (H_tool0InBase_current.p[2] - H_tool0InBase_0.p[2]))),
        KDL::Vector(0.0, 0.0, 0.0));

    // use kdlsolver to compute q_dot from twist
    kdlsolver->ik_vel(q_current, Hdot_tool0InBase_desired, qdot_command);
}

void AdmittanceController::compute_sensor_frame_admittance() {
    // sensor frame admittance
    // TODO NEXT

    // compute tool0 pose and twist in base
    kdlsolver->fk_vel(q_current, qdot_current, H_Hdot_tool0InBase_current);
}

void AdmittanceController::loop() {
    // base link admittance
    compute_admittance();

    // publish q_dot on topic
    joint_value.data.resize(6);
    joint_value.data[0] = qdot_command.data[0];
    joint_value.data[1] = qdot_command.data[1];
    joint_value.data[2] = qdot_command.data[2];
    joint_value.data[3] = qdot_command.data[3];
    joint_value.data[4] = qdot_command.data[4];
    joint_value.data[5] = qdot_command.data[5];

    std::ostringstream oss;
    oss << qdot_command.data.transpose().format(Eigen::IOFormat(
        Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "", "", "", "[", "]"));
    std::string qdot_command_str = oss.str();
    RCLCPP_INFO(get_logger(), "qdot_command: [%s]", qdot_command_str.c_str());

    pub_->publish(joint_value);
}

LifecycleCallbackReturn
AdmittanceController::on_configure(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In configure");
    (void)prev_state;

    ft_sensor_tf_ = this->get_parameter("ft_sensor_tf").as_string();
    base_tf_ = this->get_parameter("base_tf").as_string();
    joint_command_topic_ = this->get_parameter("joint_command_topic").as_string();
    ft_wrench_topic_ = this->get_parameter("ft_wrench_topic").as_string();
    joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();

    applied_axis = this->get_parameter("applied_axis").as_bool_array();
    mass = this->get_parameter("mass").as_double_array();
    damping = this->get_parameter("damping").as_double_array();
    stiffness = this->get_parameter("stiffness").as_double_array();

    pub_ = this->create_publisher<FloatMsg>(joint_command_topic_, 10);
    sub_ = this->create_subscription<WrenchMsg>(
        ft_wrench_topic_,
        10,
        std::bind(&AdmittanceController::get_wrench_current,
                  this,
                  std::placeholders::_1));
    sub_jointstate_ = this->create_subscription<JStateMsg>(
        joint_state_topic_,
        10,
        std::bind(&AdmittanceController::get_q_qdot_current,
                  this,
                  std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::duration<double>(dt),
                                     std::bind(&AdmittanceController::loop, this));
    timer_->cancel();
    oldwrench =
        KDL::Wrench(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));

    // setup KDL
    std::string urdf_file =
        ament_index_cpp::get_package_share_directory("aljnu_description") +
        "/urdf/ur5e.urdf";
    kdlsolver = std::make_unique<KDLSolver>(urdf_file, base_tf_, ft_sensor_tf_);
    q_current.resize(numjoints);
    qdot_current.resize(numjoints);
    qdot_command.resize(numjoints);

    if (false) {
        return LifecycleCallbackReturn::FAILURE;
    }
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn
AdmittanceController::on_activate(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In activate");
    timer_->reset();
    // handle pub sub activate
    rclcpp_lifecycle::LifecycleNode::on_activate(prev_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn
AdmittanceController::on_cleanup(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In cleanup");
    (void)prev_state;
    sub_.reset();
    pub_.reset();
    sub_jointstate_.reset();
    timer_.reset();
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn
AdmittanceController::on_deactivate(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In deactivate");
    timer_->cancel();

    joint_value.data.resize(6);
    joint_value.data[0] = 0.0;
    joint_value.data[1] = 0.0;
    joint_value.data[2] = 0.0;
    joint_value.data[3] = 0.0;
    joint_value.data[4] = 0.0;
    joint_value.data[5] = 0.0;
    pub_->publish(joint_value);

    rclcpp_lifecycle::LifecycleNode::on_deactivate(prev_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn
AdmittanceController::on_shutdown(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In shutdown");
    (void)prev_state;
    sub_.reset();
    pub_.reset();
    sub_jointstate_.reset();
    timer_.reset();
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn
AdmittanceController::on_error(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "In error");
    (void)prev_state;
    return LifecycleCallbackReturn::SUCCESS;
}
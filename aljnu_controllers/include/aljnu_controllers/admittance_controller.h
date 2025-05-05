#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

class KDLSolver {
    private:
        urdf::Model robot_model;
        KDL::Tree tree;
        KDL::Chain chain;

        KDL::Rotation _R;
        KDL::Vector _p;
        KDL::Frame _frame;
        KDL::Jacobian _jac;
        KDL::JntArray _jary;

        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
        std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver;
        std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_pos_solver;
        std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver;
        std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver;

    public:
        KDLSolver(/* args */);
        ~KDLSolver();
};

KDLSolver::KDLSolver(/* args */) {
    bool sucs_model = kdl_parser::treeFromUrdfModel(robot_model, tree);
    bool sucs = tree.getChain("base", "tip", chain);
    unsigned int num_j = chain.getNrOfJoints();

    fk_pos_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
    fk_vel_solver = std::make_unique<KDL::ChainFkSolverVel_recursive>(chain);
    ik_pos_solver = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain);
    ik_vel_solver = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain);
    jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(chain);
}

KDLSolver::~KDLSolver() {
}

using FloatMsg = std_msgs::msg::Float64MultiArray;
using LifecycleCallbackReturn = rclcpp_lifecycle::LifecycleNode::CallbackReturn;

class AdmittanceController : rclcpp_lifecycle::LifecycleNode {
    private:
        std::string ft_sensor_tf_;
        std::string base_tf_;
        std::string joint_command_topic_;
        double filter_;

        FloatMsg::SharedPtr joint_value;

        rclcpp::Publisher<FloatMsg>::SharedPtr pub_ = this->create_publisher<FloatMsg>("/command", 10);

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

AdmittanceController::AdmittanceController() : LifecycleNode("admittance_controller") {
    RCLCPP_INFO(get_logger(), "In Constructor");

    this->declare_parameter<std::string>("ft_sensor_tf", "/ft_sensor");
    this->declare_parameter<std::string>("base_tf", "/base_link");
    this->declare_parameter<std::string>("joint_command_topic", "/command");
}

AdmittanceController::~AdmittanceController() {
}

LifecycleCallbackReturn AdmittanceController::on_configure(const rclcpp_lifecycle::State &prev_state) {
}
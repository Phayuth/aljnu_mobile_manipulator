#include "aljnu_controllers/kdlsolver.h"

KDLSolver::KDLSolver(const std::string &urdf_file, const std::string &base,
                     const std::string &tip) {
    bool loaded_success = robot_model.initFile(urdf_file);
    if (loaded_success) {
        std::cout << "successfully loaded model." << std::endl;
    }

    bool built_success = kdl_parser::treeFromUrdfModel(robot_model, tree);
    if (built_success) {
        std::cout << "successfully built tree." << std::endl;
    }

    bool chain_success = tree.getChain(base, tip, chain);
    if (chain_success) {
        std::cout << "successfully built chain" << std::endl;
    }

    numjoints = chain.getNrOfJoints();

    fk_pos_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
    fk_vel_solver = std::make_unique<KDL::ChainFkSolverVel_recursive>(chain);
    ik_pos_solver = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain);
    ik_vel_solver = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain);
    jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(chain);
}

KDLSolver::~KDLSolver() {
}

bool KDLSolver::fk_pos(KDL::JntArray &q, KDL::Frame &H) {
    if (q.rows() != numjoints) {
        return false;
    }
    return fk_pos_solver->JntToCart(q, H);
}

bool KDLSolver::ik_pos(KDL::Frame &H, KDL::JntArray &q_init,
                       KDL::JntArray &q_out) {
    if (q_init.rows() != numjoints) {
        return false;
    }
    return ik_pos_solver->CartToJnt(q_init, H, q_out);
}

bool KDLSolver::jacobian(KDL::Jacobian &jac, KDL::JntArray &q) {
    if ((q.rows() != numjoints) || jac.columns() != numjoints ||
        jac.rows() != numjoints) {
        return false;
    }
    return jac_solver->JntToJac(q, jac);
}

bool KDLSolver::ik_vel(KDL::JntArray q_init, KDL::Twist &v_tip,
                       KDL::JntArray &q_dot) {
    if (q_dot.rows() != numjoints) {
        return false;
    }
    return ik_vel_solver->CartToJnt(q_init, v_tip, q_dot);
}

bool KDLSolver::_verify_fk_pos() {
    KDL::JntArray q(6);
    q.data << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    KDL::Frame H;
    fk_pos(q, H);
    std::cout << H << std::endl;
    return true;
}

bool KDLSolver::_verify_ik_pos() {
    KDL::JntArray q(6);
    q.data << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    KDL::Rotation rin(
        0.280, 0.957, 0.077, 0.075, 0.058, -0.995, -0.957, 0.284, -0.056);
    KDL::Vector tin(0.112, -0.275, 0.294);
    KDL::Frame pin(rin, tin);
    KDL::JntArray qout(6);

    ik_pos(pin, q, qout);
    std::cout << qout.data << std::endl;
    return true;
}

bool KDLSolver::_verify_jac() {

    KDL::JntArray q_init(6);
    q_init.data << 0.0, 1.3, 2.2, 1.4, 0.4, 0.1;

    KDL::JntArray q_dot(6);
    q_dot.data << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

    // vtip = Jac @ q_dot
    KDL::Jacobian jac(6);
    jacobian(jac, q_init);

    KDL::Twist v_tip_computed;
    for (unsigned int i = 0; i < 6; ++i) {
        v_tip_computed[i] = 0.0;
        for (unsigned int j = 0; j < numjoints; ++j) {
            v_tip_computed[i] += jac(i, j) * q_dot(j);
        }
    }

    KDL::JntArray q_dot_computed(6);
    ik_vel(q_init, v_tip_computed, q_dot_computed);
    std::cout << q_dot_computed.data << std::endl;
    std::cout << "Is q_dot the same ? " << (q_dot_computed == q_dot) << std::endl;
    return (q_dot_computed == q_dot);
}
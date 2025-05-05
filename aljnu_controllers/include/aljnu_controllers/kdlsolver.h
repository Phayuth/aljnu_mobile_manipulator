#ifndef ALJNU_CONTROLLER_KDL_SOLVER_H
#define ALJNU_CONTROLLER_KDL_SOLVER_H

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
        unsigned int numjoints;

        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
        std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver;
        std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_pos_solver;
        std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver;
        std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver;

    public:
        KDLSolver(const std::string &urdf_file, const std::string &base, const std::string &tip);
        ~KDLSolver();

        bool fk_pos(KDL::JntArray &q, KDL::Frame &H);
        bool fk_vel();
        bool ik_pos(KDL::Frame &H, KDL::JntArray &q_init, KDL::JntArray &q_out);
        bool ik_vel(KDL::JntArray q_init, KDL::Twist &v_tip, KDL::JntArray &q_dot);
        bool jacobian(KDL::Jacobian &jac, KDL::JntArray &q);

        bool _verify_fk_pos();
        bool _verify_fk_vel();
        bool _verify_ik_pos();
        bool _verify_jac();
};

#endif
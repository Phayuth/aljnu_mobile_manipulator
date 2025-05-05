#include <iostream>
#include <string>

#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/frames_io.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Expect xml file to parse" << std::endl;
        return -1;
    }
    urdf::Model robot_model;
    if (!robot_model.initFile(argv[1])) {
        std::cerr << "Could not generate robot model" << std::endl;
        return false;
    }

    KDL::Tree my_tree;
    if (!kdl_parser::treeFromUrdfModel(robot_model, my_tree)) {
        std::cerr << "Could not extract kdl tree" << std::endl;
        return false;
    }

    std::cout << " ======================================" << std::endl;
    std::cout << " Tree has " << my_tree.getNrOfSegments() << " link(s) and a root link" << std::endl;
    std::cout << " ======================================" << std::endl;
    std::cout << " Num Joints " << my_tree.getNrOfJoints() << std::endl;

    KDL::Chain chain;
    if (!my_tree.getChain("ur5e_base_link", "onrobotsg_tip", chain)) {
        std::cerr << "Could not extract chain from tree" << std::endl;
        return -1;
    }
    std::cout << " ======================================" << std::endl;
    std::cout << " Num Segs" << chain.getNrOfSegments() << " link(s) and a root link" << std::endl;
    std::cout << " ======================================" << std::endl;
    std::cout << " Num Joints " << chain.getNrOfJoints() << std::endl;

    KDL::ChainFkSolverPos_recursive c(chain);

    KDL::JntArray qin(6);
    qin.data << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    std::cout << "joint value : " << qin.data << std::endl;

    KDL::Frame pout;
    c.JntToCart(qin, pout);

    std::cout << pout << std::endl;

    KDL::Rotation rin(0.280,  0.957,  0.077, 0.075,  0.058, -0.995, -0.957,  0.284, -0.056);
    KDL::Vector tin(0.112, -0.275, 0.294);
    KDL::Frame pin(rin, tin);

    KDL::JntArray qout(6);

    KDL::ChainIkSolverPos_LMA cik(chain);
    auto s = cik.CartToJnt(qin, pin, qout);

    std::cout << "solved : " << s << std::endl;
    std::cout << "joint ik value : \n" << qout.data << std::endl;
    return 0;

}
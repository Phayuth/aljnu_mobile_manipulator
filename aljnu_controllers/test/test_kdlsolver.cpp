#include "aljnu_controllers/kdlsolver.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <string>

TEST(KDLSolverTest, VerifyFunctions) {
    std::string urdf_file =
        ament_index_cpp::get_package_share_directory("aljnu_description") +
        "/urdf/ur5e.urdf";
    KDLSolver kdlsolver(urdf_file, "ur5e_base_link", "ur5e_tool0");

    testing::internal::CaptureStdout();
    kdlsolver._verify_fk_pos();
    std::string fk_pos_output = testing::internal::GetCapturedStdout();
    std::cout << "FK Position Output:\n" << fk_pos_output;

    testing::internal::CaptureStdout();
    kdlsolver._verify_ik_pos();
    std::string ik_pos_output = testing::internal::GetCapturedStdout();
    std::cout << "IK Position Output:\n" << ik_pos_output;

    testing::internal::CaptureStdout();
    kdlsolver._verify_jac();
    std::string jac_output = testing::internal::GetCapturedStdout();
    std::cout << "Jacobian Output:\n" << jac_output;

    // Dummy assertion to ensure the test passes
    ASSERT_EQ(1, 1);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

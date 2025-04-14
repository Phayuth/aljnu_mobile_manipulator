#include "aljnu_controllers/backstepping_controller.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char const *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryBacksteppingController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
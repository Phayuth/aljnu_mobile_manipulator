#include "aljnu_controllers/position_controller.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionController>());
    rclcpp::shutdown();
    return 0;
}
#include "viperxcontroller/viperx_controller.hpp"

int main(int argc, char* argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    // Create a ROS 2 node
    auto const node = std::make_shared<rclcpp::Node>(
    "viperx_controller",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create an instance of ViperXController
    auto viper_controller = std::make_shared<viperx::ViperXController>(node, "interbotix_arm");

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}

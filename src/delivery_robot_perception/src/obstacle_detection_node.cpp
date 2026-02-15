#include "delivery_robot_perception/obstacle_detection.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<delivery_robot_perception::ObstacleDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

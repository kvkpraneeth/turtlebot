#include "turtlebot_vision/vision.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vision>());
    rclcpp::shutdown();
    return 0;
}


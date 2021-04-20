#include "turtlebot_control/control.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<control>());
    rclcpp::shutdown();
    return 0;
}


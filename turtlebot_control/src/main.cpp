#include "turtlebot_control/control.h"
#include <rclcpp/executors.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::executors::MultiThreadedExecutor exe;

    std::shared_ptr<control> slidingModeControllerNode = std::make_shared<control>();

    exe.add_node(slidingModeControllerNode->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}


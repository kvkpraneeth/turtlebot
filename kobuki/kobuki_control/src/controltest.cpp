#include "stdlib.h"
#include "kobuki_control/core.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char * argv[])
{

	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor exe;

	std::shared_ptr<KobukiControl> k = std::make_shared<KobukiControl>
															("/dev/kobuki");

	exe.add_node(k->get_node_base_interface());

	exe.spin();

	rclcpp::shutdown();

	return 0;
}

#include "turtlebot/interface/interface.hpp"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor exe;

	turtlebot::interface::driver driver("/dev/kobuki");

	exe.add_node(driver.get_node_base_interface());

	exe.spin();

	rclcpp::shutdown();

	return 0;
}
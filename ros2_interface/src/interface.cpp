#include "turtlebot/interface/interface.hpp"
#include <chrono>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <kobuki_core/parameters.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>

using namespace turtlebot::interface;

interface::interface(const std::string &device) : Node("interface")
{
	kobuki::Parameters parameters;

    parameters.device_port = device;
    parameters.sigslots_namespace = "/kobuki";
	parameters.enable_acceleration_limiter = enable_acceleration_limiter;

	if(this->enable_acceleration_limiter)
	{
		parameters.angular_acceleration_limit = angular_acceleration_limit;
		parameters.angular_deceleration_limit = angular_deceleration_limit;
		parameters.linear_acceleration_limit = linear_acceleration_limit;
		parameters.linear_deceleration_limit = linear_decelaration_limit;
	}

	parameters.battery_capacity = battery_capacity;
	parameters.battery_dangerous = battery_dangerous;
	parameters.battery_low = battery_low;
	
	kobuki.init(parameters);

    try
    {
    	kobuki.init(parameters);
    }
    catch (ecl::StandardException &e)
    {
    	std::cout << e.what();
    }

	VelSub = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 100, std::bind(&interface::VelSubCallback, this, std::placeholders::_1));

	OdometryTimer = create_wall_timer(std::chrono::duration<double, std::ratio<1,1000>>(10), std::bind(&interface::OdometryTimerCallback, this));

	OdomPub = create_publisher<nav_msgs::msg::Odometry>("/odom", 100);

	this->Pose.setZero();

}

void interface::VelSubCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	kobuki.setBaseControl(msg->linear.x, msg->angular.z);
}

void interface::OdometryTimerCallback()
{
	Eigen::Vector3d poseUpdates, poseUpdatesRates;
	this->kobuki.updateOdometry(poseUpdates, poseUpdatesRates);
}
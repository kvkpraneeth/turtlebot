#include "kobuki_control/core.h"
#include <chrono>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <kobuki_core/parameters.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>

KobukiControl::KobukiControl(const std::string &device) 
		: Node("KobukiHandler")
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

	VelSub = create_subscription<geometry_msgs::msg::Twist>
					("cmd_vel", 100, std::bind(&KobukiControl::VelSubCallback, 
						this, std::placeholders::_1));

	EncoderTimer = create_wall_timer
					(std::chrono::duration<double, std::ratio<1,1000>>(10), 
					 	std::bind(&KobukiControl::EncoderTimerCallback, this));

	OdomPub = create_publisher<nav_msgs::msg::Odometry>("/enc_odom", 100);

}

void KobukiControl::VelSubCallback
			(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	kobuki.setBaseControl(msg->linear.x, msg->angular.z);
}

void KobukiControl::EncoderTimerCallback(){
	std::cout << kobuki.getCoreSensorData().left_encoder << ", " << kobuki.getCoreSensorData().right_encoder << std::endl;
}

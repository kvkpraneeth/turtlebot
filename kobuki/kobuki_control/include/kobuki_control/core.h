#ifndef CORE_H
#define CORE_H

#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <iostream>
#include <kobuki_core/parameters.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/publisher_factory.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <string>

#include <ecl/time.hpp>
#include <ecl/command_line.hpp>

#include <kobuki_core/kobuki.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class KobukiControl : public rclcpp::Node
{

	public:
		
		KobukiControl(const std::string &device); 
		
		void VelSubCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

		void EncoderTimerCallback();

	protected:

		bool enable_acceleration_limiter = false;
		double battery_capacity = 16.5;
		double battery_low = 14.0;
		double battery_dangerous = 13.2;

		double angular_acceleration_limit;
		double angular_deceleration_limit;
		double linear_acceleration_limit;
		double linear_decelaration_limit;

		const double ticks_per_mm = 11.7;

	private:
		
		kobuki::Kobuki kobuki;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr VelSub;
		rclcpp::TimerBase::SharedPtr EncoderTimer;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr OdomPub;

};

#endif

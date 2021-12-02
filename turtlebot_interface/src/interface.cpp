#include "turtlebot/turtlebot_interface/interface.hpp"

namespace turtlebot{

	namespace interface{

		driver::driver(ros::NodeHandlePtr& nh)
		{
			kobuki::Parameters parameters;

			parameters.device_port = "/dev/kobuki";
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

			sub = nh->subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &driver::velocityCB, this);

			pub = nh->advertise<nav_msgs::Odometry>("/odom", 10);

			JointStatePub = nh->advertise<sensor_msgs::JointState>("/joint_states", 10);

			timer = nh->createTimer(ros::Duration(0.1), &driver::odomPub, this);

			this->x = 0;
			this->y = 0;
			this->theta = 0;
		}

		void driver::velocityCB(const geometry_msgs::Twist::ConstPtr& msg)
		{
			this->kobuki.setBaseControl(msg.get()->linear.x, msg.get()->angular.z);
		}

		void driver::odomPub(const ros::TimerEvent& event)
		{

			ecl::LegacyPose2D<double> poseUpdates;
			ecl::linear_algebra::Vector3d poseUpdateRates;

			sensor_msgs::JointState joint_state;

			double leftWheel, leftWheelRate, rightWheel, rightWheelRate;

			joint_state.header.frame_id = "";
			joint_state.header.seq = this->seq + 1;
			joint_state.header.stamp = ros::Time::now();

			joint_state.name.reserve(2);
			joint_state.name.push_back("wheel_right_joint");
			joint_state.name.push_back("wheel_left_joint");

			this->kobuki.updateOdometry(poseUpdates, poseUpdateRates);

			this->kobuki.getWheelJointStates(leftWheel, leftWheelRate, rightWheel, rightWheelRate);

			joint_state.position.reserve(2);
			joint_state.position.push_back(leftWheel);
			joint_state.position.push_back(rightWheel);

			this->theta += poseUpdates.heading();
			this->x += poseUpdates.x() * std::cos(theta);
			this->y += poseUpdates.x() * std::sin(theta);

			nav_msgs::Odometry msg_;

			msg_.pose.pose.position.x = this->x;
			msg_.pose.pose.position.y = this->y;

			tf2::Quaternion quat;
			quat.setRPY(0, 0, theta);

			msg_.pose.pose.orientation.w = quat.getW();
			msg_.pose.pose.orientation.x = quat.getX();
			msg_.pose.pose.orientation.y = quat.getY();
			msg_.pose.pose.orientation.z = quat.getZ();

			JointStatePub.publish(joint_state);
			pub.publish(msg_);

		}
	}
}

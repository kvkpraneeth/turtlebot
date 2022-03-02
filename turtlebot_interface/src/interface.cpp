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

			timer = nh->createTimer(ros::Duration(0.1), &driver::odomPub, this);

			this->x = 0;
			this->y = 0;
			this->theta = 0;

			if(nh->hasParam("use_imu_heading"))
			{
				nh->getParam("use_imu_heading", use_imu_heading);
			}

			if(nh->hasParam("base_frame"))
			{
				nh->getParam("base_frame", this->base_frame);
			}

			if(nh->hasParam("odom_frame"))
			{
				nh->getParam("odom_frame", this->odom_frame);
			}

			if(nh->hasParam("velocity_topic"))
			{
				nh->getParam("velocity_topic", this->velocity_topic);
			}

			if(nh->hasParam("odom_topic"))
			{
				nh->getParam("odom_topic", this->odom_topic);
			}

			if(nh->hasParam("joint_states"))
			{
				nh->getParam("joint_states", this->joint_topic);
			}

			sub = nh->subscribe<geometry_msgs::Twist>(velocity_topic, 10, &driver::velocityCB, this);

			pub = nh->advertise<nav_msgs::Odometry>(odom_topic, 10);

			JointStatePub = nh->advertise<sensor_msgs::JointState>(this->joint_topic, 10);

		}

		void driver::velocityCB(const geometry_msgs::Twist::ConstPtr& msg)
		{
			this->kobuki.setBaseControl(msg.get()->linear.x, msg.get()->angular.z);
		}

		void driver::odomPub(const ros::TimerEvent& event)
		{

			static tf2_ros::TransformBroadcaster br;

			this->seq += 1;

			ecl::LegacyPose2D<double> poseUpdates;
			ecl::linear_algebra::Vector3d poseUpdateRates;

			sensor_msgs::JointState joint_state;

			geometry_msgs::TransformStamped transformStamped;

			double leftWheel, leftWheelRate, rightWheel, rightWheelRate;

			std_msgs::Header header;

			header.frame_id = this->odom_frame;
			header.seq = seq;
			header.stamp = ros::Time::now();

			joint_state.header.frame_id = "";
			joint_state.header.seq = header.seq;
			joint_state.header.stamp = header.stamp;

			joint_state.name.reserve(2);
			joint_state.name.push_back("wheel_right_joint");
			joint_state.name.push_back("wheel_left_joint");

			this->kobuki.getWheelJointStates(leftWheel, leftWheelRate, 
				rightWheel, rightWheelRate);

			joint_state.position.reserve(2);
			joint_state.position.push_back(leftWheel);
			joint_state.position.push_back(rightWheel);

			this->kobuki.updateOdometry(poseUpdates, poseUpdateRates);

			nav_msgs::Odometry msg_;

			msg_.header = header;
			msg_.child_frame_id = this->base_frame;

			if(!use_imu_heading){
				this->theta += poseUpdates.heading();
				msg_.twist.twist.angular.z = poseUpdateRates[2];
			}

			else if(use_imu_heading){

				ecl::Angle<double> heading;
				heading = this->kobuki.getHeading();
				heading.Radians(this->theta);
				poseUpdateRates[2] = this->kobuki.getAngularVelocity();
			}

			this->x += poseUpdates.x() * std::cos(theta);
			this->y += poseUpdates.x() * std::sin(theta);
			
			msg_.twist.twist.linear.x = poseUpdateRates[0];
			msg_.twist.twist.linear.y = poseUpdateRates[1];
			
			msg_.pose.pose.position.x = this->x;
			msg_.pose.pose.position.y = this->y;
			msg_.pose.pose.position.z = 0.0;

			tf2::Quaternion quat;
			quat.setRPY(0, 0, theta);
			msg_.pose.pose.orientation.w = quat.getW();
			msg_.pose.pose.orientation.x = quat.getX();
			msg_.pose.pose.orientation.y = quat.getY();
			msg_.pose.pose.orientation.z = quat.getZ();

			msg_.pose.covariance[0] = 0.1;
			msg_.pose.covariance[7] = 0.1;
			msg_.pose.covariance[35] = use_imu_heading ? 0.005 : 0.2;

			msg_.pose.covariance[14] = 1e10;
			msg_.pose.covariance[21] = 1e10;
			msg_.pose.covariance[28] = 1e10;

			transformStamped.header = header;
			transformStamped.child_frame_id = this->base_frame;
			transformStamped.transform.translation.x = x;
			transformStamped.transform.translation.y = y;
			transformStamped.transform.translation.z = 0.0;
			
			transformStamped.transform.rotation.x = quat.getX();
			transformStamped.transform.rotation.y = quat.getY();
			transformStamped.transform.rotation.z = quat.getZ();
			transformStamped.transform.rotation.w = quat.getW();

			br.sendTransform(transformStamped);
			JointStatePub.publish(joint_state);
			pub.publish(msg_);

		}
	}
}

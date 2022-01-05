#include "turtlebot/interface/interface.hpp"

using namespace turtlebot::interface;

driver::driver(const std::string &device) : Node("interface")
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
	
	std::cout << "Init Params" << "\n";

    try
    {
		std::cout << "Trying to Init Kobuki" << "\n";
    	kobuki.init(parameters);
    }
    catch (ecl::StandardException &e)
    {
    	std::cout << e.what();
    }

	VelSub = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 100, std::bind(&driver::VelSubCallback, this, std::placeholders::_1));

	OdometryTimer = create_wall_timer(std::chrono::duration<double, std::ratio<1,1000>>(10), std::bind(&driver::OdometryTimerCallback, this));

	OdomPub = create_publisher<nav_msgs::msg::Odometry>("/odom", 100);

	JointStatePub = create_publisher<sensor_msgs::msg::JointState>("joint_states", 100);

	br = std::make_shared<tf2_ros::TransformBroadcaster>(this);

	this->Pose.setZero();
}

void driver::VelSubCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	kobuki.setBaseControl(msg->linear.x, msg->angular.z);
}

void driver::OdometryTimerCallback()
{
	
	Eigen::Vector3d poseUpdates, poseUpdatesRates;
	this->kobuki.updateOdometry(poseUpdates, poseUpdatesRates);

	sensor_msgs::msg::JointState joint_state;

	double leftWheel, leftWheelRates, rightWheel, rightWheelRates;

	joint_state.header.frame_id = "";
	joint_state.header.stamp = now();

	joint_state.name.reserve(2);
	joint_state.name.push_back("wheel_right_joint");
	joint_state.name.push_back("wheel_left_joint");

	this->kobuki.getWheelJointStates(leftWheel, leftWheelRates, rightWheel, rightWheelRates);

	joint_state.position.reserve(2);
	joint_state.position.push_back(leftWheel);
	joint_state.position.push_back(rightWheel);

	this->Pose(2) += poseUpdates(2);
	this->Pose(0) += poseUpdates(0) * std::cos(this->Pose(2));
	this->Pose(1) += poseUpdates(0) * std::sin(this->Pose(2));

	nav_msgs::msg::Odometry odom;

	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	odom.header.stamp = now();

	odom.pose.pose.position.x = this->Pose(0);
	odom.pose.pose.position.y = this->Pose(1);

	tf2::Quaternion quat;
	quat.setRPY(0, 0, this->Pose(2));

	odom.pose.pose.orientation.w = quat.getW();
	odom.pose.pose.orientation.x = quat.getX();
	odom.pose.pose.orientation.y = quat.getY();
	odom.pose.pose.orientation.z = quat.getZ();	

	geometry_msgs::msg::TransformStamped transformStamped;
	transformStamped.header.stamp = now();
	transformStamped.header.frame_id = "odom";
	transformStamped.child_frame_id = "base_footprint";
	transformStamped.transform.translation.x = Pose(0);
	transformStamped.transform.translation.y = Pose(1);
	transformStamped.transform.translation.z = 0.0;

	transformStamped.transform.rotation.x = quat.getX();
	transformStamped.transform.rotation.y = quat.getY();
	transformStamped.transform.rotation.z = quat.getZ();
	transformStamped.transform.rotation.w = quat.getW();

	br->sendTransform(transformStamped);

	JointStatePub->publish(joint_state);
	OdomPub->publish(odom);

}
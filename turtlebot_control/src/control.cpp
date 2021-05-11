#include "turtlebot_control/control.h"
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/transform__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <turtlebot_msgs/srv/detail/twist__struct.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

double quatToYaw(geometry_msgs::msg::Pose msg)
{
    double w,x,y,z;

    w = msg.orientation.w;
    x = msg.orientation.x;
    y = msg.orientation.y;
    z = msg.orientation.z;

    double yaw;

    yaw = std::atan2(2*(w*z + x*y), (1-(2*(y*y + z*z))));

    return yaw;
}

geometry_msgs::msg::Pose yawToQuat(double yaw)
{
    geometry_msgs::msg::Pose temp;

    double roll = 0, pitch = 0;

    temp.orientation.x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    temp.orientation.y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    temp.orientation.z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    temp.orientation.w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);

    return temp;
}

control::control() : Node("Controller")
{
    //Getting the parameters required for Config.
    this->declare_parameter("inputServiceName", "srv_vel");
    this->get_parameter("inputServiceName", this->inputServiceName);

    this->declare_parameter("k1", 5.0);
    this->get_parameter("k1", this->k1);

    this->declare_parameter("k2", 2.0);
    this->get_parameter("k2", this->k2);

    this->declare_parameter<long>("frequency", 5.0);
    this->get_parameter<long>("frequency", this->frequencyLong);
    this->frequency = std::chrono::milliseconds{this->frequencyLong};
    this->frequencyComputation = frequencyLong / 1e3;

    this->declare_parameter("lookAhead", 3);
    this->get_parameter("lookAhead", this->lookAhead);

    this->declare_parameter("x", 0.0);
    this->get_parameter("x", this->robotState.position.x);

    this->declare_parameter("y", 0.0);
    this->get_parameter("y", this->robotState.position.y);

    double yaw;
    this->declare_parameter("theta", 0.0);
    this->get_parameter("theta", yaw);

    this->robotState.orientation = yawToQuat(yaw).orientation;

    //Setting up Communication required by the given config.
    //this->service = this->create_service<turtlebot_msgs::srv::Twist>(this->inputServiceName, std::bind(&control::getVel, this, std::placeholders::_1, std::placeholders::_2));      
    this->timer = this->create_wall_timer(this->frequency, std::bind(&control::slidingModeControl, this));      
    this->odomSub = this->create_subscription<nav_msgs::msg::Odometry>(this->odomTopicName, 1000, std::bind(&control::odomCB, this, std::placeholders::_1));    
    this->speedPub = this->create_publisher<geometry_msgs::msg::Twist>(this->outputTopicName, 100);
    this->speedSub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1000, std::bind(&control::getVel, this, std::placeholders::_1));
}

/*
void control::getVel(const std::shared_ptr<turtlebot_msgs::srv::Twist::Request> request, std::shared_ptr<turtlebot_msgs::srv::Twist::Response> response)
{
    this->velMsg = request->velocity;
    response->done = 1;
}
*/

void control::getVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->velMsg.linear = msg->linear;
    this->velMsg.angular = msg->angular;
}

void control::odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robotState = msg->pose.pose;
}

geometry_msgs::msg::Pose control::trajectoryGeneration()
{
    geometry_msgs::msg::Pose temp;

    temp = robotState;

    for(int i = 0; i<lookAhead; i++)
    {
        temp.position.x += (this->velMsg.linear.x * cos(quatToYaw(temp)) * this->frequencyComputation);
        temp.position.y += (this->velMsg.angular.x * sin(quatToYaw(temp)) * this->frequencyComputation);
        temp.orientation = yawToQuat(quatToYaw(robotState) + (this->velMsg.angular.z * this->frequencyComputation)).orientation;
    }

    return temp;
}

void control::slidingModeControl()
{
    geometry_msgs::msg::Pose desiredPose = this->trajectoryGeneration();

    double the = quatToYaw(desiredPose) - quatToYaw(robotState);
    double xe = (robotState.position.x - desiredPose.position.x);
    double ye = (robotState.position.y - desiredPose.position.y);

    geometry_msgs::msg::Twist speedMsg;

    speedMsg.angular.z = velMsg.angular.z - (this->k2 * tanh(the - xe + ye));
    speedMsg.linear.x = (-speedMsg.angular.z)*(xe+ye) + (velMsg.linear.x * (cos(the) + sin(the))) - (this->k1) * (tanh(xe - ye));

    speedPub->publish(speedMsg);

}

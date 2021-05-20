#include "turtlebot_control/control.h"
#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/transform__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
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
    this->declare_parameter<double>("k1", 5.0);
    this->get_parameter<double>("k1", this->k1);

    this->declare_parameter<double>("k2", 2.0);
    this->get_parameter<double>("k2", this->k2);

    this->declare_parameter<double>("frequency", 5.0);
    this->get_parameter<double>("frequency", this->frequencyDouble);

    this->frequencyComputation = frequencyDouble / 1e3;
    std::chrono::duration <double, std::ratio<1,1000>> frequency{this->frequencyDouble};

    this->declare_parameter<int>("lookAhead", 3);
    this->get_parameter<int>("lookAhead", this->lookAhead);
    
    this->declare_parameter<bool>("mode", 0);
    this->get_parameter<bool>("mode", this->mode);

    this->kpv = 0.2;
    this->kpth = 0.2;

    this->timer = this->create_wall_timer(frequency, std::bind(&control::slidingModeControl, this));      
    
    this->odomSub = this->create_subscription<nav_msgs::msg::Odometry>(this->odomTopicName, 1000, std::bind(&control::odomCB, this, std::placeholders::_1));    

    this->speedPub = this->create_publisher<geometry_msgs::msg::Twist>(this->outputTopicName, 100);
    
    this->speedSub = this->create_subscription<geometry_msgs::msg::Twist>(this->inputTopicName, 1000, std::bind(&control::getVel, this, std::placeholders::_1));

    this->poseSub = this->create_subscription<geometry_msgs::msg::Pose>(this->desiredPoseTopicName, 1000, std::bind(&control::getPose, this, std::placeholders::_1));         

    this->desiredSpeedPub = this->create_publisher<geometry_msgs::msg::Twist>(this->desiredSpeedTopicName, 100);

    robotState.position.x = 0;
    robotState.position.y = 0;
    robotState.orientation.x = 0;
    robotState.orientation.y = 0;
    robotState.orientation.z = 0;
    robotState.orientation.w = 1;

    this->prevRobotState = robotState;

}

void control::getVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    inputSignal.speed.linear = msg->linear;
    inputSignal.speed.angular = msg->angular;
}

void control::odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    
    double dx = robotState.position.x - prevRobotState.position.x;
    double dy = robotState.position.y - prevRobotState.position.y;
    double dth = quatToYaw(robotState) - quatToYaw(prevRobotState);

    if(dx > 0.01 && dy > 0.01 && dth > 0.01)
    {
        prevRobotState = robotState;
        robotState = msg->pose.pose;
    }

}

void control::getPose(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    inputSignal.pose.position = msg->position;
    inputSignal.pose.orientation = msg->orientation;
}

struct signal control::lookAheadIntegration(geometry_msgs::msg::Twist speedInput, double dt)
{
    struct signal temp;

    temp.pose = robotState;

    for(int i = 0; i<lookAhead; i++)
    {
        temp.pose.position.x += (speedInput.linear.x * cos(quatToYaw(temp.pose)) * dt);
        temp.pose.position.y += (speedInput.angular.x * sin(quatToYaw(temp.pose)) * dt);
        temp.pose.orientation = yawToQuat(quatToYaw(robotState) + (speedInput.angular.z * dt)).orientation;
    }

    temp.speed = speedInput;

    return temp;
}

struct signal control::velocityGeneration(geometry_msgs::msg::Pose poseInput, double dt)
{
    struct signal temp;
    
    double dx = poseInput.position.x - robotState.position.x;
    double dy = poseInput.position.y - robotState.position.y;
    
    temp.speed.linear.x = this->kpv*(sqrt(dx*dx + dy*dy));
    temp.speed.angular.z = this->kpth*(atan2(dy,dx));
    temp.pose = poseInput;

    RCLCPP_INFO(get_logger(), "d, %f, %f", dy, dx);

    return temp;
}

void control::slidingModeControl()
{
    
    this->get_parameter<double>("k1", this->k1);
    this->get_parameter<double>("k2", this->k2);
    this->get_parameter<double>("frequency", this->frequencyDouble);
    this->get_parameter<int>("lookAhead", this->lookAhead);
    this->get_parameter<bool>("mode", this->mode);

    double the,xe,ye;

    struct signal controlSignal;

    if(mode == 1)
    {
        controlSignal = this->lookAheadIntegration(inputSignal.speed, frequencyComputation);
    }

    if(mode == 0)
    {
        controlSignal = this->velocityGeneration(inputSignal.pose, frequencyComputation);
    }

    the = quatToYaw(controlSignal.pose) - quatToYaw(robotState);
    xe = (robotState.position.x - controlSignal.pose.position.x);
    ye = (robotState.position.y - controlSignal.pose.position.y);

    geometry_msgs::msg::Twist speedMsg;

    speedMsg.angular.z = controlSignal.speed.angular.z - (this->k2 * tanh(the - xe + ye));
    speedMsg.linear.x = (-speedMsg.angular.z)*(xe+ye) + (controlSignal.speed.linear.x * (cos(the) + sin(the))) - (this->k1) * (tanh(xe - ye));

    if (speedMsg.linear.x > 2.0)
    {
        speedMsg.linear.x = 2.0;
    }

/*
    if (abs(speedMsg.angular.z) > 1.5)
    {
        speedMsg.angular.z = 1.5;
    }
*/

    speedPub->publish(speedMsg);
    this->desiredSpeedPub->publish(controlSignal.speed);

}

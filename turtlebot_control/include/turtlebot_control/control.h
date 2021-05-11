#ifndef CONTROL_H
#define CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "turtlebot_msgs/srv/twist.hpp"
#include <chrono>
#include <geometry_msgs/msg/detail/transform__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <turtlebot_msgs/srv/detail/twist__struct.hpp>
#include "nav_msgs/msg/odometry.hpp"

class control : public rclcpp::Node
{

    //Constructor.
    public: control();

    //Crucial Names.
    private: std::string inputServiceName;
             //outputTopicName is Constant as it depends on the internal plugin.
             const std::string outputTopicName="cmd_vel_robot";
             //odomTopicName is Constant as it depends on the internal plugin.
             const std::string odomTopicName="odom";

    //Server Function.
    public: void getVel(const std::shared_ptr<turtlebot_msgs::srv::Twist::Request> request, std::shared_ptr<turtlebot_msgs::srv::Twist::Response> response);

    //Server.
    private: rclcpp::Service<turtlebot_msgs::srv::Twist>::SharedPtr service;

    //Class Variables.
    private: geometry_msgs::msg::Twist velMsg;

    //SMC Parameters.
    private: float k1,k2;
             int lookAhead;

    //Controller Timer.
    private: std::shared_ptr<rclcpp::TimerBase> timer;
    private: std::chrono::milliseconds frequency;
             long frequencyLong;
             long frequencyComputation;

    //Controller Timer Callback.
    private: void slidingModeControl();

    //Trajectory Generation.
    public: geometry_msgs::msg::Pose trajectoryGeneration();

    //Robot Variables.
    private: geometry_msgs::msg::Pose robotState;

    //Subscriptions.
    private: rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;

    //Callbacks.
    public: void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg);

};

#endif

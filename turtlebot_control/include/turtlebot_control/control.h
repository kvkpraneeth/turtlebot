#ifndef CONTROL_H
#define CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/transform__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include "nav_msgs/msg/odometry.hpp"

struct signal
{
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Twist speed;
};

class control : public rclcpp::Node
{

    //Constructor.
    public: control();

    //Crucial Names.
    private: //outputTopicName is Constant as it depends on the internal plugin.
             const std::string outputTopicName="cmd_vel_robot";
             //odomTopicName is Constant as it depends on the internal plugin.
             const std::string odomTopicName="odom";
             //InputTopicName.
             const std::string inputTopicName="cmd_vel";
             //DesiredSpeedTopicName. (-> Used For Debugging/Tuning)
             const std::string desiredSpeedTopicName="des_vel";
             //DesiredPoseTopicName.
             const std::string desiredPoseTopicName="des_pose";

    //Class Variables.
    private: struct signal inputSignal;

    //SMC Parameters.
    private: double k1,k2;
             int lookAhead;
             bool mode;

    //P Parameters.
    private: double kpv, kpth;

    //Controller Timer.
    private: std::shared_ptr<rclcpp::TimerBase> timer;
    private: double frequencyDouble;
             double frequencyComputation;

    //Controller Timer Callback.
    public: void slidingModeControl();

    //LookAhead Integration.
    public: struct signal lookAheadIntegration(geometry_msgs::msg::Twist speedInput, double dt);

    //Velocity Generation.
    public: struct signal velocityGeneration(geometry_msgs::msg::Pose poseInput, double dt);

    //Robot Variables.
    private: geometry_msgs::msg::Pose robotState;
             geometry_msgs::msg::Pose prevRobotState;

    //Subscriptions.
    private: rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
             rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr speedSub;
             rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr poseSub;

    //Callbacks.
    public: void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg);
            void getVel(const geometry_msgs::msg::Twist::SharedPtr msg);
            void getPose(const geometry_msgs::msg::Pose::SharedPtr msg);

    //Publications.
    private: rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speedPub;
             rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr desiredSpeedPub;

};

#endif

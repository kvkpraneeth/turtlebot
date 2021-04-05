#ifndef TURTLEBOT_CONTROL_H
#define TURTLEBOT_CONTROL_H

#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/service_server.h"
#include "ros/time.h"
#include "std_msgs/Float64.h"
#include "turtlebot/vel.h"
#include <cstddef>
#include <memory>
#include "realtime_tools/realtime_publisher.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_msgs/TFMessage.h"

class control
{

    public:
        
        control(ros::NodeHandle &nh_);
        
        void loop();
        
        void get_params();

        bool getvel(turtlebot::vel::Request &req, turtlebot::vel::Response &res);

        void odometry();

        nav_msgs::Odometry robot;

        double x, y, th, vx, vy, vth;

        ros::Time current_time, last_time;

    private: 

        ros::Publisher odom_broadcaster;

        void sendTransform(const std::vector<geometry_msgs::TransformStamped> &msgtf);

    protected:

        ros::NodeHandle nh;

        double wSep, wDia;

        geometry_msgs::Twist velmsg;

        ros::ServiceServer server;

        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>> left_pub_real;
        
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>> right_pub_real;

        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_real;
};

#endif

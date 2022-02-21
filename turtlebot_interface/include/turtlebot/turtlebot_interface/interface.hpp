#pragma once

#include "kobuki_driver/command.hpp"
#include "kobuki_driver/kobuki.hpp"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ecl/linear_algebra.hpp"
#include "ecl/geometry/legacy_pose2d.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "sensor_msgs/JointState.h"
#include "math.h"
#include <tf2_ros/transform_broadcaster.h>
#include "std_msgs/Header.h"

namespace turtlebot{

    namespace interface{
        
        class driver
        {

            public:

                driver(ros::NodeHandlePtr& nh);

                void velocityCB(const geometry_msgs::Twist::ConstPtr& msg);

                void odomPub(const ros::TimerEvent& event);

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
                ros::Subscriber sub;
                ros::Publisher pub;
                ros::Publisher JointStatePub;

                ros::Timer timer;

                double x, y, theta;
                int seq=0;


        };
    }
}
#pragma once

#include "ros/ros.h"
#include "memory"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

# define PI 3.14159265358979323846

namespace turtlebot{

    namespace utility{

        double quatToYaw(const nav_msgs::Odometry::ConstPtr& msg)
        {

            double w,x,y,z;

            w = msg.get()->pose.pose.orientation.w;
            x = msg.get()->pose.pose.orientation.x;
            y = msg.get()->pose.pose.orientation.y;
            z = msg.get()->pose.pose.orientation.z;

            double yaw;

            yaw = std::atan2(2*(w*z + x*y), (1-(2*(y*y + z*z))));

            return yaw;

        }
    }
}
#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include <tf2_ros/transform_listener.h>

namespace turtlebot{

    namespace pid{

        struct system
        {
            double target_value;
            double current_value;
            double prev_error=0;
            double dt;
        };

        typedef struct system system;

        struct pid
        {
            double kp;
            double kd;
            double ki;

            double upper_limit;            
            double lower_limit;
        };

        typedef struct pid pid;

        double getVal(system& system, pid& controller);

    }

    namespace control{

        class controller
        {
            public:

                controller(ros::NodeHandlePtr& nh, tf2_ros::Buffer* buffer);
                void subCB(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);

            private:

                tf2_ros::Buffer* buffer_;

                ros::Publisher velocityPub;
                ros::Subscriber fiducialSub;

                pid::pid angular_velocity;
                pid::pid linear_velocity;

        };

    }

}
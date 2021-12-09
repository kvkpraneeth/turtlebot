#include "turtlebot/turtlebot_control/control.hpp"

double turtlebot::pid::getVal(turtlebot::pid::system& system, turtlebot::pid::pid& controller)
{

    if(system.dt == 0)
    {
        return system.current_value;
    }

    double error = system.target_value - system.current_value;

    double u = error*controller.kp + controller.kd*(error-system.prev_error);

    system.prev_error = error;

    return u;

}

turtlebot::control::controller::controller(ros::NodeHandlePtr& nh, tf2_ros::Buffer* buffer)
{
    this->velocityPub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    this->fiducialSub = nh->subscribe<fiducial_msgs::FiducialTransformArray>("/fiducial_transforms", 100, &turtlebot::control::controller::subCB, this);
    this->buffer_ = buffer;
}

void turtlebot::control::controller::subCB(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
{
    
    geometry_msgs::TransformStamped transformStamped;

    geometry_msgs::Twist vel_msg;

    if(msg->transforms.empty())
    {
        vel_msg.linear.x = 0;  
    }

    else
    {
        try
        {
            transformStamped = buffer_->lookupTransform("fiducial_6", "base_link", ros::Time(0));
            vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));
            vel_msg.angular.z = 0.01 * atan2(transformStamped.transform.translation.y, transformStamped.transform.translation.x);
        }

        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            vel_msg.linear.x = 0;
        }   
    }

    velocityPub.publish(vel_msg);


}
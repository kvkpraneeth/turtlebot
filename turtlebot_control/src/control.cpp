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

turtlebot::control::controller::controller(ros::NodeHandlePtr& nh)
{
    this->velocityPub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    this->fiducialSub = nh->subscribe<fiducial_msgs::FiducialTransformArray>("/fiducials", 10,&turtlebot::control::controller::fiducialCB, this);
}

void turtlebot::control::controller::fiducialCB(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
{
}
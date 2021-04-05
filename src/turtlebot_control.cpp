#include "turtlebot/turtlebot_control.h"

void control::sendTransform(const std::vector<geometry_msgs::TransformStamped> &msgtf)
{
    tf2_msgs::TFMessage msg;

    for(std::vector<geometry_msgs::TransformStamped>::const_iterator it = msgtf.begin(); it != msgtf.end(); it++)
    {
        msg.transforms.push_back(*it);
    }
    
    this->odom_broadcaster.publish(msg);
}


control::control(ros::NodeHandle &nh_)
{
   
    this->nh = nh_;
    
    this->get_params();

    this->server = this->nh.advertiseService("cmd_vel", &control::getvel, this);

    this->velmsg.angular.x = 0;
    this->velmsg.angular.y = 0;
    this->velmsg.angular.z = 0;

    this->velmsg.linear.x = 0;
    this->velmsg.linear.y = 0;
    this->velmsg.linear.z = 0;

    this->left_pub_real.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(this->nh, "/wheel_left_controller/command", 100));
    this->right_pub_real.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(this->nh, "/wheel_right_controller/command", 100));

    this->odom_pub_real.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(this->nh, "/odom", 100));

    this->odom_broadcaster = this->nh.advertise<tf2_msgs::TFMessage>("/tf", 1000);

    this->x = 0.0;
    this->y = 0.0;
    this->th = 0.0;

    this->vx = this->velmsg.linear.x;
    this->vy = this->velmsg.linear.y;
    this->vth = this->velmsg.angular.z;

    this->current_time = ros::Time::now();
    this->last_time = ros::Time::now();
}

void control::loop()
{
    this->left_pub_real->msg_.data = (2*this->velmsg.linear.x - this->velmsg.angular.z * this->wSep)/(this->wDia);
    this->right_pub_real->msg_.data = (2*this->velmsg.linear.x + this->velmsg.angular.z * this->wSep)/(this->wDia);  
    this->odometry();
    this->odom_pub_real->unlockAndPublish();
    this->left_pub_real->unlockAndPublish();
    this->right_pub_real->unlockAndPublish();
}

void control::get_params()
{
    this->nh.getParam("wheelSeparation", this->wSep);
    this->nh.getParam("wheelDiameter", this->wDia);
}

bool control::getvel(turtlebot::vel::Request &req, turtlebot::vel::Response &res)
{
    this->velmsg = req.velmsg;    
    return true;
}

void control::odometry()
{
    this->current_time = ros::Time::now();

    this->vx = this->velmsg.linear.x;
    this->vy = this->velmsg.linear.y;
    this->vth = this->velmsg.angular.z;

    double dt = (this->current_time - this->last_time).toSec();
    double dx = (this->vx*cos(th))*dt;
    double dy = (this->vx*sin(th))*dt;
    double dth = this->vth * dt;

    this->x += dx;
    this->y += dy;
    this->th += dth;

    double zero = 0.0;

    tf2::Quaternion quat;

    quat.setRPY(this->th, zero, zero);

    this->robot.pose.pose.orientation.w = quat.w();
    this->robot.pose.pose.orientation.x = quat.x();
    this->robot.pose.pose.orientation.y = quat.y();
    this->robot.pose.pose.orientation.z = quat.z();

    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = this->x;
    odom_trans.transform.translation.y = this->y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = this->robot.pose.pose.orientation;

    std::vector<geometry_msgs::TransformStamped> odom_trans_;    

    odom_trans_.push_back(odom_trans);

    //send the transform
    this->sendTransform(odom_trans_);

    //next, we'll publish the odometry message over ROS
    this->robot.header.stamp = current_time;
    this->robot.header.frame_id = "odom";

    //set the position
    this->robot.pose.pose.position.x = this->x;
    this->robot.pose.pose.position.y = this->y;
    this->robot.pose.pose.position.z = 0.0;

    //set the velocity
    this->robot.child_frame_id = "base_footprint";
    this->robot.twist.twist.linear.x = this->vx;
    this->robot.twist.twist.linear.y = 0; // for non-holonomic robots this is always 0
    this->robot.twist.twist.angular.z = this->vth;

    this->odom_pub_real->msg_ = this->robot;

    last_time = current_time;
    
}

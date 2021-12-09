#include "turtlebot/turtlebot_control/control.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "controller");

    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

    tf2_ros::Buffer* buffer = (new tf2_ros::Buffer());

    tf2_ros::TransformListener tf2_list(*(buffer));

    turtlebot::control::controller c(nh, buffer);

    ros::spin();

    return 0;

}
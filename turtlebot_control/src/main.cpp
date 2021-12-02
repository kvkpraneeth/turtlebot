#include "turtlebot/turtlebot_control/control.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "controller");

    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

    ros::spin();

    return 0;

}
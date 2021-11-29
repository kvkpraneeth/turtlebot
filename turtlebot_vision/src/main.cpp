#include "turtlebot/turtlebot_vision/core.hpp"

int main(int argc, char * argv[])
{

    ros::init(argc, argv, "vision");

    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

    ros::spin();

    return 0;

}
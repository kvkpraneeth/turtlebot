#include "turtlebot/turtlebot_interface/core.hpp"

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "interface");
    
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

    turtlebot::interface::driver d(nh);

    ros::spin();

    return 0;

}
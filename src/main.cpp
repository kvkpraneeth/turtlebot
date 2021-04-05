#include "ros/rate.h"
#include "turtlebot/turtlebot_control.h"
#include <ros/callback_queue.h>

int main(int argc, char * argv[])
{

    ros::init(argc, argv, "controller");

    ros::NodeHandle nh;

    control c(nh);

    ros::Rate r(100.0);

    while(ros::ok())
    {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.001));
        c.loop();
        r.sleep();
    }

    return 0;
}


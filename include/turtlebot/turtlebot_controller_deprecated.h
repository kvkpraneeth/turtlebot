#include "controller_interface/controller.h"
#include "controller_interface/controller_base.h"
#include "hardware_interface/joint_command_interface.h"
#include "ros/duration.h"
#include "ros/node_handle.h"
#include <string>
#include <vector>

class joint
{

    public:

        hardware_interface::JointHandle joint;
        double gain;
        double setpoint;

};

namespace turtlebot_control{

    class controller : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public: 

            void init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n, std::vector<std::string> &joint_names)
            {
                for(std::vector<std::string>::const_iterator it = joint_names.begin(); it != joint_names.end(); it++)
                {

                    joint J;
                    J.joint = hw->getHandle(*it);
                    J.gain = 1.0;
                    J.setpoint = 1.57;
                    this->joints.push_back(J);                

                }
            }


            void update(const ros::Time& time, const ros::Duration& period)
            {
                for(std::vector<joint>::iterator it = this->joints.begin(); it != this->joints.end(); it++)
                {
                 
                    double error = it->setpoint - it->joint.getPosition();
                    double gain = it->gain;

                    it->joint.setCommand(error*gain);

                }
            }

            void starting(const ros::Time& time) {  }

            void stopping(const ros::Time& time) {  }

        private:

            std::vector<joint> joints;
    
    };

}

# Turtlebot Stack

Branches:

- ROS1 : Noetic_Real_Buggy
    - Description: Basic setup that can both drive the turtlebot in simulation and reality. Built by combining and combing out bugs from melodic development. 
    - Status: Completed.
    - Note: Not a pleasent package to setup, but works well and has nothing more to be added to.

- ROS2 : smc-sim
    - Description: An implementation of Switching based Sliding Mode Control for trajectory tracking.
    - Status: Completed.

- ROS1 : ros1_devel
    - Description: A bug free from scratch implementation for using turtlebot2(Kobuki) in simulation and reality with ROS Noetic.
    - Status: In Progress; A better and more complete ros wrapper being built.
    - Note: Purely for research purposes.
 
- ROS2 : ros2_devel
    - Description: A bug free from scratch implementation for using turtlebot2(Kobuki) in simulation and reality with ROS2 Foxy.
    - Status: In Progress; A better and more complete ros2 wrapper being built.
    - Note: Purely for research purposes.

- ROS1 : mapping
    - Description: An attempt to interface SLAM with the wrapper previously built with ROS and Astra Camera.
    - Status: In Progress; tf setup and tuning left out.

- ROS1 : shepherd
    - Description: Robot follows robot using Fiducial markers; Two turtlebots in reality and ROS1 Noetic without any communication b/w them. Initial attempt at basic swarm behavioural algorithms
    - Status: Completed
    - https://youtu.be/w6hWpJQhLT0
    

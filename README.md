# Turtlebot Stack

Branches:

- ROS1 : Noetic
    - Description: Basic setup and an attempt to create everything from scratch
    - Status: Abandoned.
    - Reason: A waste of time to create everything from scratch. Not worth it.

- ROS2 : SMCVG
    - Description: An attempt at implementation of the paper that presented a vision based local planner.
    - Status: Abandoned.
    - Reason: The model worked almost perfectly, which made it clear that simualtions are too good to be true, hence of no use.

- ROS2 : rrt-smc
    - Description: An implementation of Switching based Sliding Mode Control (Non Linear)
    - Status: Success.
    - Note: Misleading branch name, switching branch to make a better iteration of the mentioned combination in the name.

- ROS2 : stack2
    - Description: An implementation of a Non Linear Control model developed in 'rrt-smc' wrapped with Navigation 2 stack.
    - Status: In Progress.
    - Note: The reason Nav2 stack was selected is to have a tested architecture in use. Making my own makes no sense and is a waste of time.

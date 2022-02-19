# Robot exploration (ROS)

## Description

A mini-project where we help [TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) robot explore an unknown environment. The robot is called Burger and you can see a picture of Burger below.


<img src="images/turtlebot3_burger.png" alt="drawing" width="400"/>

The ability to perform autonomous exploration is essential for an autonomous system operating in unstructured or unknown environments where it is hard or even impossible to describe the environment beforehand.

## System description

Uses an _exploration_ node, based on [receding horizon "next-best-view" (RH-NBV)](https://ieeexplore.ieee.org/abstract/document/7487281). For collision avoidance, a _collision avoidance_ node, based on [the obstacle-restriction method (ORM)](https://ieeexplore.ieee.org/abstract/document/1545546) and [pure pursuit](https://apps.dtic.mil/docs/citations/ADA255524) (for smooth control) ensures safe path following. In order for Burger to localize itself, a _SLAM_ node, based on [Hector SLAM](https://wiki.ros.org/hector_slam), which does mapping and localization. it also makes use of a [_costmap_2d_](https://wiki.ros.org/costmap_2d) node in order to make the exploration and collision avoidance simpler. [Gazebo](http://gazebosim.org/).

The plan is to create a _controller_ node that is using the exploration node and the collision avoidance node in order to move Burger around in the environment.

[LINK TO SIMULATION VIDEO](https://www.youtube.com/watch?v=bMZBmDRP2XM)

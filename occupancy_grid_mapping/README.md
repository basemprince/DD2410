
# OCCUPANCY GRID MAPPING (ROS)

Mapping is one of the core competencies of truly autonomous robots. Autonomous robots can use maps in a number of different ways, 
search and rescue robots can use them to make sure they search through the whole building instead of moving in between the same rooms over and over, 
autonomous cars use them in order to find a path that leads to the desired location, multicopters can use the maps to localize themself in order to s
tay in the air.

In many situations we cannot assume that the robot can be given a map in advance. Even if there are maps available, such as blueprints for a building, 
they are not always useful for the robot and might be incorrect (too old, building collapsed, etc). Therefore it is of great benefit if the robot can 
construct a map by itself from scratch, which is exactly what we will do in this assignment.


Occupancy grid mapping is one of many mapping algorithms. Here the world is represented as a grid, where each cell of the grid corresponds to an area in the world.
The value of the cell can tell us if the area is free, occupied, unknown, or something else.

The occupancy grid is characterized by the number of cells and the resolution. More cells means that it is possible to map a larger area. 
In this project, we worked with a 2D occupancy grid but 3D grid maps are often used as well.

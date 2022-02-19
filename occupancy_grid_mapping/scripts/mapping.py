#!/usr/bin/env python3

"""
    # {Basem Shaker}
    # {9752 2382 8601 2726}
    # {bshaker@kth.se}
"""

# Python standard library
from math import cos, sin, atan2, fabs,sqrt

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate
import time
from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:f
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = [grid_map.get_origin().position.x,grid_map.get_origin().position.y]
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()

        robot = [pose.pose.position.x,pose.pose.position.y]   
        robot_pos = [robot[0]-origin[0],robot[1]-origin[1]]
        robot_pos_r = [int(x/resolution) for x in robot_pos]
        obstacle_location = [0,0]
        obstacle_array = []
        obstacle_x = []
        obstacle_y = []
        for i, rangeData in enumerate(scan.ranges):
            if (rangeData <= scan.range_min or rangeData >= scan.range_max):
                continue
            angle = i * scan.angle_increment
            bearing = scan.angle_min + angle
            current_angle = bearing + robot_yaw
            x_location = int(((rangeData * cos(current_angle)) + robot_pos[0])/ resolution)
            y_location = int(((rangeData * sin(current_angle)) + robot_pos[1])/ resolution)
            obstacle_location = [x_location, y_location]
            obstacle_array.append(obstacle_location)
            obstacle_x.append(x_location)
            obstacle_y.append(y_location)

            free_spaces = self.raytrace(robot_pos_r,obstacle_location)
            for free_space in free_spaces:
                self.add_to_map(grid_map,free_space[0],free_space[1],self.free_space)



        for obstacle in obstacle_array:
            self.add_to_map(grid_map,obstacle[0],obstacle[1],self.occupied_space)

        """
        For C only!
        Fill in the update correctly below.
        """ 
        #print(obstacle_array)
        min_x , max_x = min(obstacle_x) , max(obstacle_x)
        min_y , max_y = min(obstacle_y) , max(obstacle_y)

        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated    
        update.x = min_x
        # The minimum y index in 'grid_map' that has been updated
        update.y = min_y
        # Maximum x index - minimum x index + 1
        update.width = max_x - min_x +1
        # Maximum y index - minimum y index + 1
        update.height = max_y - min_y +1
        # The map data inside the rectangle, in row-major order.
        update.data = []
        #print("x,y,w,h:",update.x,update.y,update.width,update.height)
        
        for y in range(min_y,max_y+1):
            for x in range (min_x,max_x+1):
                itemValue = grid_map[x,y]
                update.data.append(itemValue)

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """


        """
        Fill in your solution here
        """
        map_height = grid_map.get_height()
        map_width = grid_map.get_width()

        for width in range(map_width):
            for height in range(map_height):
                if grid_map[width,height] == self.occupied_space:
                    circleArray = circle_list([width,height],self.radius)
                    for circle in circleArray:
                        if grid_map[circle[0], circle[1]]  != self.occupied_space:                                
                            self.add_to_map(grid_map, circle[0], circle[1], self.c_space)

        # Return the inflated map
        return grid_map


def circle_list (point,radius):
    x, y = point
    circleArray = []

    for i in range (x-radius,x+radius):
        for j in range (y-radius,y+radius):
            if (sqrt((x-i)**2+(y-j)**2) <= radius):
                circleArray.append([i,j])

    return circleArray
                

                    

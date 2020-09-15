#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Basem Shaker}
# {9752 2382 8601 2726}
# {bshaker@kth.se}
import math
from dubins import *
import time
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np


plt.style.use('ggplot')

plot = False
dt = 0.01


def safe(x,y,car):
    for obs in car.obs:
        distance = math.sqrt((x - obs[0])**2 + (y - obs[1])**2)
        if ((distance < obs[2] + 0.1)) or not ((car.xlb < x < car.xub and car.ylb < y < car.yub) ) :
            return False
    return True



def possiblePositions (car,current_node):
    currentX,currentY,currentTheta = current_node[0],current_node[1],current_node[2]

    allowedPoints = []
    for angle in (0,math.pi/4,-math.pi/4):
        x,y,theta = step(car,currentX,currentY,currentTheta,angle,dt)
        result = [x,y,theta,angle]
        if safe(result[0],result[1],car):
            while result[2] >= math.pi:
                result[2] -= 2*math.pi
            while result[2] <= -2*math.pi:
                result[2] += math.pi  
            allowedPoints.append(result)
    return allowedPoints


class aStarNode():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return round(self.position[0],2) == round(other.position[0],2) and round(self.position[1],2)== round(other.position[1],2) and round(self.position[2],2)== round(other.position[2],2) 
    
    def distance(self,other):
        return math.sqrt((self.position[0] - other.position[0])**2 + (self.position[1]-other.position[1])**2)


def astar(car,start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = aStarNode(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = aStarNode(None, end)
    end_node.g = end_node.h = end_node.f = 0
    within = 1
    
    
    open_list = []
    closed_list = []

    open_list.append(start_node)

 
    nodesX = []
    nodesY = []

    if plot:
        fig, ax = plt.subplots(nrows=1, ncols=1, sharex=True, sharey=True)

    while len(open_list) > 0:
        #time.sleep(0.05)
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        open_list.sort(key=lambda x: x.f)
        #print(open_list)
        for index, item in enumerate(open_list):
            #print("index: ",index,"item: ",item)
            if item.f < current_node.f:
                current_node = item
                current_index = index
        

        open_list.pop(current_index)
        closed_list.append(current_node)
        #print("open_list: ", len(open_list),"closed_list: ",len(closed_list))
        # Found the goal
        #print(current_node.position)
        if current_node.distance(end_node) <= within:
            controls=[0]
            times=[0,1]
            current = current_node
            while current is not None:    
                #print(controls)  
                #print(current.position[2])
                controls.append(current.position[3])
                times.append(times[-1] + dt)
                current = current.parent
            return controls [::-1]  , times # Return reversed controls and times

        # Generate children
        adjacentPoints = possiblePositions(car,current_node.position)
        #print(adjacentPoints)
        children = []
        nodesX.append(current_node.position[0])
        nodesY.append(current_node.position[1])
        #print(current_node.position[s0],current_node.position[1], math.degrees(current_node.position[2]))

        if (plot):
            for ob in car.obs:
                ax.add_patch(plt.Circle((ob[0], ob[1]), ob[2], facecolor="gray", edgecolor="k"))
            ax.plot(car.x0, car.y0, 'kx')
            ax.plot(car.xt, car.yt, 'kx')
            ax.plot(nodesX, nodesY, 'k-')
            ax.set_xlim(car.xlb, car.xub)
            ax.set_ylim(car.ylb, car.yub)
            ax.set_aspect("equal")
            plt.pause(0.001)


        for new_position in adjacentPoints:
            new_node = aStarNode(current_node, new_position)
            children.append(new_node)
        for child in children:
            for closed_child in closed_list:
                if child == closed_child:
                    continue
            child.g = current_node.g +1
            child.h = (((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2))
            child.f = child.g + child.h
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue
            open_list.append(child)


def solution(car):

    ''' <<< write your code below >>> '''
    controls=[0]
    times=[0,1]

    startPoint = (car.x0,car.y0,0,0)
    endPoint = (car.xt,car.yt)

    
    controls,times=astar(car,startPoint, endPoint)
    print(controls,times)
    #print("length" ,len(controls),len(times))
    ''' <<< write your code above >>> '''
    return controls, times


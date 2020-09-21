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

plot1 = True
plot2 = not plot1

if True:
    plot1 = False
    plot2 = False 
dt = 1


def safe(x,y,car):
    for obs in car.obs:
        distance = math.sqrt((x - obs[0])**2 + (y - obs[1])**2)
        if ((distance < obs[2] + 0.3)) or not ((car.xlb < x < car.xub and car.ylb < y < car.yub) ) :
            return False
    return True



def possiblePositions (car,current_node,previous_occurences,mapGrid):
    currentX,currentY,currentTheta = current_node[0],current_node[1],current_node[2]
    check = True
    allowedPoints = []
    upcomingPoint = []
    #print("new node")
    for angle in [0,math.pi/4,-math.pi/4]:
        x,y,theta = step(car,currentX,currentY,currentTheta,angle,dt)
        nextX,nextY,nextTheta = step(car,x,y,theta,angle,dt)
        #print ("angle: " , angle, "x,y,z: ", x,y,theta)
        result = [x,y,theta,angle]
        nextResult = [nextX,nextY]
        allowedPoints.append(result)
        upcomingPoint.append(nextResult)
        # if safe(result[0],result[1],car):
        #     while result[2] >= math.pi:
        #         result[2] -= 2*math.pi
        #     while result[2] <= -2*math.pi:
        #         result[2] += math.pi  
        #     for item in previous_occurences:
        #         if round(item.position[0],1) == round(x,1) and round(item.position[1],1)== round(y,1):
        #             check =False
        #             break 
        #         check = True
        #     #print(check)
        #     if(check):
        #         allowedPoints.append(result)

    return allowedPoints, upcomingPoint


class aStarNode():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        distance = math.sqrt((self.position[0] - other.position[0])**2 + (self.position[1]-other.position[1])**2)
        return distance >= dt
        #return round(self.position[0],1) == round(other.position[0],1) and round(self.position[1],1)== round(other.position[1],1) #and round(self.position[2],1)== round(other.position[2],1) 
    
    def distance(self,other):
        return math.sqrt((self.position[0] - other.position[0])**2 + (self.position[1]-other.position[1])**2)


def gridCreator(car,start,end):

    map_grid = []

    for x in range (0,int(car.xt),dt):
        for y in range (0,int(car.yt),dt):
            map_grid.append([x,y,x+dt,y+dt,0])
    #print (map_grid)
    return map_grid


def astar(car,start, end,controls,times,mapGrid):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = aStarNode(None, start)


    start_node.g = start_node.h = start_node.f = 0
    end_node = aStarNode(None, end)
    end_node.g = end_node.h = end_node.f = 0
    within = 1
    
    
    open_list = []
    closed_list = []
    for gridBlock in mapGrid:
        if gridBlock[2]>start_node.position[0]> gridBlock[0] and gridBlock[3]>start_node.position[1]> gridBlock[1]:
            gridBlock[4]=1
    open_list.append(start_node)
    nodesX = []
    nodesY = []

    if plot1 or plot2:
        plt.clf()
        fig, ax = plt.subplots(nrows=1, ncols=1, sharex=True, sharey=True)

    while len(open_list) > 0:
        #time.sleep(2)
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        #open_list.sort(key=lambda x: x.f)
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
            current = current_node
            while current is not None:    
                #print(controls)  
                #print(current.position[2])
                controls.append(current.position[3])
                times.append(times[-1] + dt)
                current = current.parent
            return controls [::-1]  , times # Return reversed controls and times

        # Generate children
        adjacentPoints,nextPoint = possiblePositions(car,current_node.position,open_list,mapGrid)
        #print(adjacentPoints)
        children = []
        nodesX.append(current_node.position[0])
        nodesY.append(current_node.position[1])
        #print(current_node.position[s0],current_node.position[1], math.degrees(current_node.position[2]))

        if (plot1):
            for ob in car.obs:
                ax.add_patch(plt.Circle((ob[0], ob[1]), ob[2], facecolor="gray", edgecolor="k"))
            ax.plot(car.x0, car.y0, 'kx')
            ax.plot(car.xt, car.yt, 'kx')
            ax.plot(nodesX, nodesY, 'k-')
            if(len(open_list)>0):
                ax.scatter(open_list[-1].position[0],open_list[-1].position[1],color='red')
            if(len(closed_list)>0):
                ax.scatter(closed_list[-1].position[0],closed_list[-1].position[1],color='grey')
            ax.set_xlim(car.xlb, car.xub)
            ax.set_ylim(car.ylb, car.yub)
            ax.set_aspect("equal")
            plt.pause(0.001)


        for new_position,upcoming_position in zip(adjacentPoints,nextPoint):

            new_node = aStarNode(current_node, new_position)

            if not safe(new_position[0],new_position[1],car) and (new_):
                continue

            if (plot2):
                for ob in car.obs:
                    ax.add_patch(plt.Circle((ob[0], ob[1]), ob[2], facecolor="gray", edgecolor="k"))
                ax.plot(car.x0, car.y0, 'kx')
                ax.plot(car.xt, car.yt, 'kx')
                #ax.plot(nodesX, nodesY, 'k-')
                ax.scatter(new_position[0],new_position[1])
                ax.set_xlim(car.xlb, car.xub)
                ax.set_ylim(car.ylb, car.yub)
                ax.set_aspect("equal")
                plt.pause(0.001)
                
            children.append(new_node)
            for gridBlock in mapGrid:
                if gridBlock[2]>new_node.position[0]> gridBlock[0] and gridBlock[3]>new_node.position[1]> gridBlock[1]:
                    gridBlock[4]=1

        for child in children:
            # for closed_child in closed_list:
            #     if child == closed_child:
            #         continue
            
            #child.g = abs(current_node.position[0] - child.position[0]) + abs(current_node.position[1] - child.position[1])
            #child.g = current_node.g + 1
            #child.g = math.sqrt(((child.position[0] - current_node.position[0]) ** 2) + ((child.position[1] - current_node.position[1]) ** 2))
            child.g = current_node.g + child.distance(current_node)
            child.h = 1.3*math.sqrt(((end_node.position[0] - upcoming_position[0]) ** 2) + ((end_node.position[1] - upcoming_position[1]) ** 2))
            #child.h = 1.2*child.distance(end_node)
            #child.h = abs(child.position[0] - end_node.position[0]) + abs(child.position[1] - end_node.position[1])
            child.f = child.g + child.h



            #print(current_node.position[s0],current_node.position[1], math.degrees(current_node.position[2]))

            # if child.f < sum(c.f for c in children)/len(children):
            #     continue

            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue
            open_list.append(child)


def solution(car):

    ''' <<< write your code below >>> '''
    controls=[0]
    times=[0,dt]

    startPoint = (car.x0,car.y0,0,0)
    endPoint = (car.xt,car.yt)

    mapGrid=gridCreator(car,startPoint,endPoint)
    controls,times=astar(car,startPoint, endPoint,controls,times,mapGrid)
    print(controls,times)
    #print("length" ,len(controls),len(times))
    ''' <<< write your code above >>> '''
    return controls, times


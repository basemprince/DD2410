#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Basem Shaker}
# {9752 2382 8601 2726}
# {bshaker@kth.se}

from dubins import *
import math
from queue import *
#import matplotlib as mpl
#import matplotlib.pyplot as plt

within = 0.5
dt = 0.01
plot = False

class MyPriorityQueue(PriorityQueue):
    def __init__(self):
        PriorityQueue.__init__(self)
        self.counter = 0

    def put(self, item, priority):
        PriorityQueue.put(self, (priority, self.counter, item))
        self.counter += 1

    def get(self, *args, **kwargs):
        _, _, item = PriorityQueue.get(self, *args, **kwargs)
        return item


# class Node():
#     def __init__(self, xPos, yPos, headingAngle, control,time):
#         self.xPos = xPos
#         self.yPos = yPos
#         self.headingAngle = headingAngle
#         self.control = control
#         self.time = time


def bf_search(car, controls,times):
    # current_state = Node(car.x0,car.y0,0,controls,times)
    node_object = [car.x0,car.y0,0,controls,times]
    node_cost = math.sqrt((car.x0 - car.xt)**2 + (car.y0 - car.yt)**2)
    fifo = MyPriorityQueue()
    fifo.put(node_object,node_cost)
    closePoint = []
    nodesX=[]
    nodesY=[]
#    if plot:
#        fig, ax = plt.subplots(nrows=1, ncols=1, sharex=True, sharey=True)
    while not fifo.empty():
        xPos,yPos,headingAngle,controls,times = fifo.get()
        nodesX.append(xPos)
        nodesY.append(yPos)
        # if (plot):
        #     for ob in car.obs:
        #         ax.add_patch(plt.Circle((ob[0], ob[1]), ob[2], facecolor="gray", edgecolor="k"))
        #     ax.plot(car.x0, car.y0, 'kx')
        #     ax.plot(car.xt, car.yt, 'kx')
        #     ax.plot(nodesX, nodesY, 'k-')
        #     ax.set_xlim(car.xlb, car.xub)
        #     ax.set_ylim(car.ylb, car.yub)
        #     ax.set_aspect("equal")
        #     plt.pause(0.001)
        distance_from_goal = math.sqrt((xPos - car.xt)**2 + (yPos - car.yt)**2)
        if distance_from_goal <= within:
            return controls, times
        for stearingAngle in [0,math.pi/4, -math.pi/4]:
            xPosn, yPosn, headingAnglen, controlsn, timesn,cost, check = possiblePositions(car,xPos,yPos,headingAngle,stearingAngle,replace(controls),replace(times))
            if check and not [round(xPosn,1), round(yPosn,1), round(headingAnglen,1)] in closePoint:
                closePoint.append([round(xPosn,1), round(yPosn,1), round(headingAnglen,1)])
                fifo.put([xPosn, yPosn, headingAnglen, controlsn, timesn],cost)
    return controls,times


def possiblePositions(car,xPos,yPos,headingAngle,stearingAngle,controls,times):
    cost = 0
    for i in range(100 if stearingAngle == 0 else 157):
        xPos, yPos, headingAngle = step(car,xPos, yPos, headingAngle, stearingAngle)
        while headingAngle >= 2*math.pi:
            headingAngle -= math.pi
        while headingAngle <= -math.pi:
            headingAngle += 2*math.pi  
        times.append(times[-1] + dt)
        controls.append(stearingAngle)
        distance_from_goal = math.sqrt((xPos - car.xt)**2 + (yPos - car.yt)**2)
        if notSafe(xPos,yPos,car):
            return 0, 0, 0, controls, times, 999999, False
        if distance_from_goal <= within:
           return xPos, yPos, headingAngle, controls, times, 0, True
    cost = distance_from_goal
    return xPos, yPos, headingAngle, controls, times, cost, True

def notSafe(xPos,yPos,car):
    for obs in car.obs:
        distance = math.sqrt((xPos - obs[0])**2 + (yPos - obs[1])**2)
        if ((distance <= obs[2] + 0.05)) or not ((car.xlb < xPos < car.xub and car.ylb < yPos < car.yub) ) :
            return True
    return False

   
def replace(array):
    new = []
    for current in array:
        new.append(current)
    return new

def solution(car):
    controls=[0]
    times=[0,dt]
    controls, times = bf_search(car, controls,times)
    return controls, times

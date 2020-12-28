#! /usr/bin/env python3

"""
    # Basem Shaker
    # bshaker@kth.se
"""

import math

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]
    l = [0.07,0.3,0.35]
    
    x_fix = x-l[0]
    a = x_fix**2 + y**2 - l[1]**2- l[2]**2
    b = 2 * l[1] * l[2]
    q2= math.acos(a/b)
    
    c = l[2]* math.sin(q2)
    d = l[1] + (l[2]* math.cos(q2))
    q1 = math.atan2(y,x_fix) - math.atan2(c,d)
    q3 = z
    q=[q1,q2,q3]

    return q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """

    return q

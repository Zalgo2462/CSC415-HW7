"""
                        ***meclib.py***
Convenience methods for driving a mecanum drive robot
"""
import math
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from sympy import *
from memoization import *

def packageMecanumFK(fr, br, bl, fl, r, l1, l2):
    """
    packages fr, br, bl, fl, r, l1, and l2 into a Float64MultiArray
    fr: front right wheel speed
    br: back right wheel speed
    bl: back left wheel speed
    fl: front left wheel speed
    l1: the distance between the left and right wheels
    l2: the distance between the front and back wheels
    """
    data = Float64MultiArray(data=[])
    data.layout = MultiArrayLayout()
    data.layout.dim = [MultiArrayDimension()]
    data.layout.dim[0].label = "Parameters"
    data.layout.dim[0].size = 7
    data.layout.dim[0].stride = 1
    data.data = [fr, br, bl, fl, r, l1, l2]
    return data

def mechanumIK(currTheta, xDot, yDot, thetaDot, r, l1, l2):
    """
    computes the wheel velocities given world velocities and the current orientation
    currTheta: the current craft orientation
    xDot: x component of the velocity
    yDot: y component of the velocity
    thetaDot: change in orientation
    r: wheel radius
    l1: the distance between the left and right wheels
    l2: the distance between the front and back wheels
    """
    proj = np.matrix(
        [
            [1, -1, -(l1 + l2)],
            [1, 1, (l1 + l2)],
            [1, 1, -(l1 + l2)],
            [1, -1, (l1 + l2)]
        ]
    )
    rot = np.matrix(
        [
            [math.cos(currTheta), math.sin(currTheta), 0],
            [-math.sin(currTheta), math.cos(currTheta), 0],
            [0, 0, 1]
        ]
    )
    vel = np.matrix(
        [
            [xDot],
            [yDot],
            [thetaDot],
        ]
    )
    wheels = (1/r) * proj.dot(rot.dot(vel))
    return wheels[0,0], wheels[1,0], wheels[2,0], wheels[3,0]


def getAngleTo(currTheta, robotX, robotY, destX, destY):
    """
    returns the smallest angle to the destination
    currTheta: current robot theta
    robotX: current robot x position
    robotY: current robot y position
    destX: destination x position
    destY: destination y position
    """
    headingVecX = math.cos(currTheta)
    headingVecY = math.sin(currTheta)
    diffVecX = destX - robotX
    diffVecY = destY - robotY
    diffDist = math.sqrt(diffVecX * diffVecX + diffVecY * diffVecY)
    dot = headingVecX * diffVecX + headingVecY * diffVecY
    cross = headingVecX * diffVecY - diffVecX * headingVecY
    if cross == 0: return 0
    theta = math.acos(dot / diffDist) # magHeadingVec = 1
    return -theta if cross < 0 else theta

def getDistanceTo(robotX, robotY, destX, destY):
    """
    returns the distance to a point
    robotX: current robot x position
    robotY: current robot y position
    destX: destination x position
    destY: destination y position
    """
    dx = destX - robotX
    dy = destY - robotY
    return math.sqrt(dx * dx + dy * dy)

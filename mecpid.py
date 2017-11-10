#!/usr/bin/env python
"""
                          ***mecpid.py***
Connect to stdr_simulator to implement pid path following for a mecanum robot.
""" 

import rospy
import time
import math
import sys
import numpy as np
import matplotlib.pyplot as plt

from meclib import *
from stdr_robot_data import *
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Range

def main(wheelR, l1, l2):
    rospy.init_node('pidBot')

    # Set up publisher
    mecFKPub = rospy.Publisher('/robot0/kinematic_params', Float64MultiArray, queue_size=1)

    # Set up dt
    DTReader = getDTReader()
    rospy.Subscriber("/robot0/dt", Float64, DTReader)

    # Set up pose
    poseReader = getPoseReader()
    rospy.Subscriber("/robot0/pose2D", Pose2D, poseReader)

    # Wait for ROS
    time.sleep(1)

    # Set up the path
    t = np.linspace(0, 2.0*np.pi, 24)
    pathSize = 10
    xLocs = pathSize * np.sin(t) + 12
    yLocs = pathSize * np.sin(2*t) + 12

    # Track the current goal
    i = 0
    destX = xLocs[i]
    destY = yLocs[i]

    # Locations of the obstacles
    obj1X = 6
    obj1Y = 12
    obj2X = 18
    obj2Y = 12

    # Control variables
    v = 1 # linear velocity
    oldErrTheta = 0
    oldOldErrTheta = 0
    oldHeadingControl = 0

    kpHeading = 1
    kiHeading = 0
    kdHeading = 0

    while not rospy.is_shutdown():
        startTime = time.time()
        currTheta = poseReader.pose.theta
        currX = poseReader.pose.x
        currY = poseReader.pose.y

        destDist = getDistanceTo(currX, currY, destX, destY)
        obj1Dist = getDistanceTo(currX, currY, obj1X, obj1Y)
        obj2Dist = getDistanceTo(currX, currY, obj2X, obj2Y)

        if (destDist < 0.2):
            if i == len(t) - 1:
                break
            i += 1
            destX = xLocs[i]
            destY = yLocs[i]

        errTheta  = getAngleTo(currTheta, currX, currY, obj1X, obj1Y) if obj1Dist < obj2Dist else getAngleTo(currTheta, currX, currY, obj2X, obj2Y)

        newHeadingControl = oldHeadingControl + \
        kpHeading * (errTheta - oldErrTheta) + \
        kiHeading * (errTheta + oldErrTheta) + \
        kdHeading * (errTheta - 2 * oldErrTheta + oldOldErrTheta)

        destAngle = math.atan2(destY - currY, destX - currX)
        xDot = math.cos(destAngle) * v
        yDot = math.sin(destAngle) * v

        fl, fr, bl, br = mechanumIK(currTheta, xDot, yDot, newHeadingControl, wheelR, l1, l2)

        oldOldErrTheta = oldErrTheta
        oldErrTheta = errTheta
        oldHeadingControl = newHeadingControl

        newStep = packageMecanumFK(fr, br, bl, fl, wheelR, l1, l2)
        time.sleep(DTReader.dt - (time.time() - startTime))
        mecFKPub.publish(newStep)

    mecFKPub.publish(packageMecanumFK(0, 0, 0, 0, wheelR, l1, l2))

if __name__ == '__main__':
    try:
        wheelR = 0.1
        l1 = 0.4
        l2 = 0.5
        main(wheelR, l1, l2)
    except rospy.ROSInterruptException:
        pass


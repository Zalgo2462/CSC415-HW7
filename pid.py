#!/usr/bin/env python
"""
                          ***pid.py***
Connect to stdr_simulator to implement pid path following for a dd robot.
"""

import rospy
import time
import math
import sys
import numpy as np

from ddlib import *
from stdr_robot_data import *
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Range

def main(wheelR, robotR):
    rospy.init_node('pidBot')

    # Set up publisher
    ddFKPub = rospy.Publisher('/robot0/kinematic_params', Float64MultiArray, queue_size=1)

    # Set up dt
    DTReader = getDTReader()
    rospy.Subscriber("/robot0/dt", Float64, DTReader)

    # Set up pose
    poseReader = getPoseReader()
    rospy.Subscriber("/robot0/pose2D", Pose2D, poseReader)

    time.sleep(1)

    t = np.linspace(0, 2.0*np.pi, 24)
    pathSize = 10
    xLocs = pathSize * np.sin(t) + 12
    yLocs = pathSize * np.sin(2*t) + 12

    i = 0
    destX = xLocs[i]
    destY = yLocs[i]

    v = 13
    oldErrTheta = 0
    oldErrDist = 0
    oldP1 = v
    oldP2 = v
    kHeadingOnly = 5.5
    kHeadingClose = 5
    kSpeedClose = 0.1

    while not rospy.is_shutdown():
        print "Dest"
        print destX
        print destY
        startTime = time.time()
        currTheta = poseReader.pose.theta
        currX = poseReader.pose.x
        currY = poseReader.pose.y
        errTheta = getAngleTo(currTheta, currX, currY, destX, destY)
        errDist = getDistanceTo(currX, currY, destX, destY)
        newP1 = None
        newP2 = None
        print "New - old theta %f" % (errTheta - oldErrTheta)
        if (errDist < 0.2):
            ddFKPub.publish(packageDDFK(0, 0, wheelR, robotR))
            if i == len(t) - 1:
                break
            i += 1
            destX = xLocs[i]
            destY = yLocs[i]
            continue
        elif (True and errDist > 1):
            print "A"
            newP1 = oldP1 + kHeadingOnly * (errTheta - oldErrTheta)
            newP2 = oldP2 - kHeadingOnly * (errTheta - oldErrTheta)
        else:
            print "B"
            newP1 = oldP1 + kSpeedClose * (
                v * (errDist - oldErrDist) +
                kHeadingClose * (errTheta * errDist - oldErrTheta * oldErrDist)
            )
            newP2 = oldP2 + kSpeedClose * (
                v * (errDist - oldErrDist) -
                kHeadingClose * (errTheta * errDist - oldErrTheta * oldErrDist)
            )
        oldErrTheta = errTheta
        oldErrDist = errDist
        oldP1 = newP1
        oldP2 = newP2
        newStep = packageDDFK(newP1, newP2, wheelR, robotR)
        time.sleep(DTReader.dt - (time.time() - startTime))
        ddFKPub.publish(newStep)

if __name__ == '__main__':
    try:
        wheelR = 0.1
        robotR = 0.2
        main(wheelR, robotR)
    except rospy.ROSInterruptException:
        pass


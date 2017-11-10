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

    v = 3
    oldErrTheta = 0
    oldOldErrTheta = 0
    oldHeadingControl = 0

    kpHeading = 4
    kiHeading = 0.0005
    kdHeading = 0.05

    while not rospy.is_shutdown():
        print "Dest (%f, %f)" % (destX, destY)
        startTime = time.time()
        currTheta = poseReader.pose.theta
        currX = poseReader.pose.x
        currY = poseReader.pose.y
        distance = getDistanceTo(currX, currY, destX, destY)

        if (distance < 0.2):
            if i == len(t) - 1:
                break
            i += 1
            destX = xLocs[i]
            destY = yLocs[i]

        errTheta = getAngleTo(currTheta, currX, currY, destX, destY)
        newHeadingControl = oldHeadingControl + \
        kpHeading * (errTheta - oldErrTheta) + \
        kiHeading * (errTheta + oldErrTheta) + \
        kdHeading * (errTheta - 2 * oldErrTheta + oldOldErrTheta)

        newP1 = (1/wheelR) * (v + robotR * newHeadingControl)
        newP2 = (1/wheelR) * (v - robotR * newHeadingControl)

        oldOldErrTheta = oldErrTheta
        oldErrTheta = errTheta
        oldHeadingControl = newHeadingControl

        print "Error: %f" % (errTheta)
        print "Error Diff: %f" % (errTheta - oldErrTheta)
        print "Error Sum: %f" % (errTheta + oldErrTheta)
        print "Control: %f" % (newHeadingControl)
        print "Wheel Speeds: (%f, %f)" %(newP1, newP2)
        print ""

        newStep = packageDDFK(newP1, newP2, wheelR, robotR)
        time.sleep(DTReader.dt - (time.time() - startTime))
        ddFKPub.publish(newStep)

    ddFKPub.publish(packageDDFK(0, 0, wheelR, robotR))

if __name__ == '__main__':
    try:
        wheelR = 0.1
        robotR = 0.2
        main(wheelR, robotR)
    except rospy.ROSInterruptException:
        pass


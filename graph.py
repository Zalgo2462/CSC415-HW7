#!/usr/bin/env python
'''
    Utility that listens to pose and plots the points obtained.
'''

import matplotlib.pyplot as plt
import rospy
import time
from ddlib import *
from geometry_msgs.msg import Pose2D
from stdr_robot_data import *
from std_msgs.msg import Float64


def main():
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim([0, 24])
    ax.set_ylim([0, 24])
    rospy.init_node('graph')
    poseReader = getPoseReader()
    rospy.Subscriber("/robot0/pose2D", Pose2D, poseReader)
    DTReader = getDTReader()
    rospy.Subscriber("/robot0/dt", Float64, DTReader)
    time.sleep(2)

    while not rospy.is_shutdown():
        startTime = time.time()

        ax.arrow(poseReader.pose.x, poseReader.pose.y, math.cos(poseReader.pose.theta), math.sin(poseReader.pose.theta),  head_width=0.2, head_length=0.25, fc='k', ec='k')
        ax.plot([poseReader.pose.x], [poseReader.pose.y], marker='o', markersize=3, color="red")
        #ax.relim()
        #ax.autoscale_view()
        plt.draw()
        # Sleep is in place to limit the amount of points graphed.
        plt.pause(DTReader.dt - (time.time() - startTime))


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

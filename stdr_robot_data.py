from memoization import *
from geometry_msgs.msg import Pose2D
import rospy

def getSonarReader():
    @static_vars(range=None)
    def sonarReader(data):
        sonarReader.range = data.range
    return sonarReader

def getPoseReader():
    @static_vars(pose=None)
    def poseReader(data):
        poseReader.pose = data
    return poseReader

def getDTReader():
    @static_vars(dt=None, rate=None)
    def DTReader(data):
        DTReader.dt = data.data
        DTReader.rate = 1.0 / data.data
    return DTReader

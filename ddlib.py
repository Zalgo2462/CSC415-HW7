"""
                        ***ddlib.py***
Convenience methods for driving a differential drive robot
"""
import math
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from sympy import *
from memoization import *

def packageDDFK(w1, w2, r, l):
    """
    packages w1, w2, r, and l into a Float64MultiArray
    w1: wheel speed 1 seen as p1 in other places (for phi 1)
    w2: wheel speed 2 seen as p2 in other places (for phi 2)
    r: wheel radius
    l: robot radius
    """
    data = Float64MultiArray(data=[])
    data.layout = MultiArrayLayout()
    data.layout.dim = [MultiArrayDimension()]
    data.layout.dim[0].label = "Parameters"
    data.layout.dim[0].size = 4
    data.layout.dim[0].stride = 1
    data.data = [w1, w2, r, l]
    return data

@memoized
def ddIK(x, y, t, r, L):
    """
    Computes the inverse kinematics for a dd robot.
    x: a function of t describing the path's x values
    y: a function of t describing the path's y values
    t: a sympy symbol parameterizing x and y
    r: the radius of the robot wheels
    L: the radisu of the robot
    """
    dx = diff(x, t)
    ddx = diff(dx, t)
    dy = diff(y, t)
    ddy = diff(dy, t)
    v = sqrt(dx * dx + dy * dy)
    k = (dx * ddy - dy * ddx) / (v**3)
    dPhi_1 = simplify((v / r) * (k * L + 1))
    dPhi_2 = simplify((v / r) * (-k * L + 1))
    return (dPhi_1, dPhi_2)

def circleCurve(h, k, t, r, v):
    """
    returns a (x,y) curve parameterized by t for the constants
    h, k, v, and r
    h: x coord center
    k: y coord center
    t: t value for parameterization
    r: radius of the circle
    v: angular velocity about the circle
    """
    x = h + r * cos(v * t)
    y = k + r * sin(v * t)
    return x, y

@static_vars(
    __rSym=symbols('R'),
    __vSym=symbols('v'),
    __tSym=symbols('t')
)
def genDDCircle(r, L, R, time, cw, simRate):
    """
    returns the constant wheel velocities to travel a circle of radius R
    this has a warm up time of one call in order to generate the IK for a circle
    r: wheel radius
    L: robot raidus
    R: circle radius
    time: how long to tkae
    cw: clockwise or counter clockwise
    simRate: simulation rate
    """
    # Divide the total time by 1 / RATE (the time interval)
    numSteps = int(time / (1.0 / simRate) + 0.5)

    # Derive angular speed
    angularSpeed = 2 * math.pi / time
    if cw:
        angularSpeed *= -1

    # Shorten the static variable names,
    # we use static variables in order to make memoize the algebra
    rSym = genDDCircle.__rSym
    vSym = genDDCircle.__vSym
    tSym = genDDCircle.__tSym

    # Set up the model, this will be memoized
    curve_x, curve_y = circleCurve(0, 0, tSym, rSym, vSym)

    # Memoized
    c_p1, c_p2 = ddIK(curve_x, curve_y, tSym, r, L)

    # Plug r and angular speed in
    c_p1, c_p2 = (
        c_p1.evalf(subs={rSym: R, vSym: angularSpeed}),
        c_p2.evalf(subs={rSym: R, vSym: angularSpeed})
    )

    return ([c_p1] * numSteps, [c_p2] * numSteps)

def genDDLine(r, distance, time, simRate):
    """
    generates the dd robot wheel speeds needed to drive a distance at a rate
    r: wheel radius
    distance: distance to travel
    time: how long ot take
    simRate: simulation rate
    """
    numSteps = int(time / (1.0 / simRate) + 0.5)
    wheelSpeed = distance / (r * time)
    return ([wheelSpeed] * numSteps, [wheelSpeed] * numSteps)

def genDDRotation(r, L, theta, time, simRate):
    """
    generates the dd robot wheel speeds needed to turn in plac in a given time
    r: wheel radius
    L: robot radius
    theta: angle to turn
    time: time taken to turn
    simRate: simulation rate
    """
    # I derived this geometrically
    numSteps = int(time / (1.0 / simRate) + 0.5)
    wheelSpeed = (theta * L) / (r * time)
    return ([wheelSpeed] * numSteps, [-wheelSpeed] * numSteps)


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


def genDDTurnTo(r, L, currTheta, robotX, robotY, destX, destY, time, simRate):
    """
    generates a turn towards a point (destX, destY) over a given time
    r: wheel radius
    L: robot radius
    currTheta: current robot theta
    robotX: current robot x position
    robotY: current robot y position
    destX: destination x position
    destY: destination y position
    time: time taken to turn
    simRate: simulation rate
    """
    diffTheta = getAngleTo(currTheta, robotX, robotY, destX, destY)
    return genDDRotation(r, L, diffTheta , time, simRate)

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


def genDDGoTo(r, L, currTheta, robotX, robotY, destX, destY, turnTime, linearTime, simRate):
    """
    generates a turn and a line to get to a point
    r: wheel radius
    L: robot radius
    currTheta: current robot theta
    robotX: current robot x position
    robotY: current robot y position
    destX: destination x position
    destY: destination y position
    turnTime: time taken to turn
    linearTime: time taken to travel the line to the point
    simRate: simulation rate
    """
    turn_p1, turn_p2 = genDDTurnTo(r, L, currTheta, robotX, robotY, destX, destY, turnTime, simRate)
    dx = destX - robotX
    dy = destY - robotY
    distance = getDistanceTo(robotX, robotY, destX, destY)
    line_p1, line_p2 = genDDLine(r, distance, linearTime, simRate)
    turn_p1.extend(line_p1)
    turn_p2.extend(line_p2)
    return turn_p1, turn_p2

def segment(offset, percentage, path):
    """
    retrieves a segment of a path
    offset: beginning offset in percentage
    percentage: the portion to take
    """
    l = len(path)
    start = int(offset * l)
    end = int((offset + percentage) * l)
    return path[start:end]

def reversePath(path):
    """
    reverses a path by negating the wheel velocities and playing it backwards
    path: the path to reverse
    """
    return [-1 * x for x in path[::-1]]

def travel(p1, p2, r, L, simRate, ddFKPub):
    """
    publishes wheel velocities for a dd robot
    p1: right wheel angular velocity
    p2: left wheel angular velocity
    r:  wheel radius
    L: robot radius
    simRate: simulation rate
    ddFKPub: ros publisher for dd kinematics
    """
    rate = rospy.Rate(simRate)
    x = min(len(p1), len(p2))

    for i in xrange(x):
        ddFKPub.publish(packageDDFK(p1[i], p2[i], r, L))
        rate.sleep()
    # Stop
    ddFKPub.publish(packageDDFK(0, 0, r, L))
    return


def goAround(r, L, R, clockwise, stepAngle, stepTime, stopFuncs, DTReader, ddFKPub, angle=None):
    """
    moves the robot forward checking conditions along the way, may optionally stop 
    after a distance
    r:  wheel radius
    L: robot radius
    R: circle radius
    clockwise: which way to turn
    stepAngle: how long to move before checking stopFuncs/ the distance travelled
    stepTime: how long it takes to move stepAngle
    stopFuncs: a list of bool functions; goForward will stop if a function is true
    DTReader: a DTReader provided by stdr_robot_data used for rate limiting
    ddFKPub: differential drive forward kinematic publisher
    angle: an optional angle to stop travelling after
    """
    fullCircleSteps = (2 * math.pi) / (stepAngle)
    fullCircleTime = fullCircleSteps * stepTime

    travelled = 0
    stopFuncs = stopFuncs[:]
    if angle != None:
        stopFuncs.append(lambda:travelled >= angle)
    while not reduce(lambda x,y : x or y, [x() for x in stopFuncs], False):
        p1, p2 = genDDCircle(r, L, R, fullCircleTime, clockwise, DTReader.rate)
        p1, p2 = (segment(0, (1.0 / fullCircleSteps), p1), segment(0, (1.0 / fullCircleSteps), p2))
        travel(p1, p2, r, L, DTReader.rate, ddFKPub)
        travelled += stepAngle
    return


def goForward(r, L, stepDist, stepTime, stopFuncs, DTReader, ddFKPub, distance=None):
    """
    moves the robot forward checking conditions along the way, may optionally stop
    after a distance
    r:  wheel radius
    L: robot radius
    stepDist: how long to move before checking stopFuncs/ the distance travelled
    stepTime: how long it takes to move stepDist
    stopFuncs: a list of bool functions; goForward will stop if a function is true
    DTReader: a DTReader provided by stdr_robot_data used for rate limiting
    ddFKPub: differential drive forward kinematic publisher
    distance: an optional distance to stop travelling after
    """
    travelled = 0
    stopFuncs = stopFuncs[:]
    if distance != None:
        stopFuncs.append(lambda:travelled >= distance)
    while not reduce(lambda x,y : x or y, [x() for x in stopFuncs], False):
        forward_1, forward_2 = genDDLine(r, stepDist, stepTime, DTReader.rate)
        travel(forward_1, forward_2, r, L, DTReader.rate, ddFKPub)
        travelled += stepDist
    return

def goBackward(r, L, stepDist, stepTime, stopFuncs, DTReader, ddFKPub, distance=None):
    """
    moves the robot backward checking conditions along the way, may optionally stop
    after a distance
    r:  wheel radius
    L: robot radius
    stepDist: how long to move before checking stopFuncs/ the distance travelled
    stopFuncs: a list of bool functions; goForward will stop if a function is true
    DTReader: a DTReader provided by stdr_robot_data used for rate limiting
    ddFKPub: differential drive forward kinematic publisher
    distance: an optional distance to stop travelling after
    """
    travelled = 0
    stopFuncs = stopFuncs[:]
    if distance != None:
        stopFuncs.append(lambda:travelled >= distance)
    while not reduce(lambda x,y : x or y, stopFuncs, False):
        forward_1, forward_2 = genDDLine(r, stepDist, stepTime, DTReader.rate)
        backward_1, backward_2 = (reversePath(forward_1), reversePath(forward_2))
        travel(backward_1, backward_2, r, L, DTReader.rate, ddFKPub)
        travelled += stepDist
    return

#! /usr/bin/env python3

import rospkg
import yaml
rospack = rospkg.RosPack()
packagePath = rospack.get_path('subharmAPF')

import matplotlib.pyplot as plt
import math
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from Lidar import *
from Control import *

rospack = rospkg.RosPack()
packagePath = rospack.get_path('subharmAPF')
with open(packagePath + '/config.yaml', 'r') as file: config = yaml.safe_load(file)
LASER_A = config['A_COEF']['LASER']

LASER_K = config['K_COEF']['LASER']
TARGET_K = config['K_COEF']['TARGET']

SAFE_DISTANCE = config['SAFE_DISTANCE']
DANGER_DISTANCE = config['DANGER_DISTANCE']
DISTANCE = config['SAMPLE_DISTANCE']
V_K = config['V_K']
MAX_LIDAR_DISTANCE = config['MAX_LIDAR_DISTANCE']
DANGER_FORCE = LASER_A * LASER_K * math.exp(-LASER_K * DANGER_DISTANCE)
USE_TRADITION = config['USE_TRADITION']

if not USE_TRADITION:
    LASER_K = 1 / DANGER_DISTANCE / DANGER_DISTANCE


def rad_to_deg(angle):
    return angle * 180 / math.pi

def lasarField(msgScan, odomMsg, plotFlag=False):
    """_summary_

    Args:
        msgScan (LaserScan): message get from robots' lidar 
        odomMsg (Odometry): message of the robot
        plotFlag (Bool) : whether to plot the field or not
        
    Returns:
        force: magnitude of the repulsive force
        angle: angle of the repulsive force
    """
    # Get robot position and orientation
    ( myx , myy ) = getPosition(odomMsg)
    theta = getRotation(odomMsg)
    # Get lidar scan
    ( lidar, angles ) = lidarScan(msgScan)
    if lidar.min() >= SAFE_DISTANCE: return 0, 0
    
    angles += rad_to_deg(theta)
    
    index = lidar.argmin()
    obs_x = lidar[index] * math.cos(math.radians(angles[index])) + odomMsg.pose.pose.position.x
    obs_y = lidar[index] * math.sin(math.radians(angles[index])) + odomMsg.pose.pose.position.y
    
    distToObs = getDistance(obs_x, myx, obs_y, myy)
    if USE_TRADITION: force = LASER_K * (1/distToObs - 1/SAFE_DISTANCE) / distToObs / distToObs
    else: force = LASER_A * math.exp( - LASER_K * distToObs * distToObs)
    angle = math.atan2( myy - obs_y, myx - obs_x )
        
    if (plotFlag):
        # Show field!!!!!
        x = np.linspace(-10, 10, 100) 
        y = np.linspace(-10, 10, 100)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros((100, 100))
        
        for i in range(100):
            for j in range(100):
                myDist = getDistance(X[i, j], obs_x, Y[i, j], obs_y)      
                if USE_TRADITION: Z[i, j] = 0.5 * LASER_K * ((1/myDist - 1/SAFE_DISTANCE) ** 2)
                else: Z[i, j] = LASER_A * math.exp(-LASER_K * myDist)
        
        fig = plt.figure(figsize=(10, 6))
        ax = fig.add_subplot(111, projection='3d')
        
        surf = ax.plot_surface(X, Y, Z, cmap="Blues_r", cstride=1, rstride=1)
        fig.colorbar(surf)
        plt.savefig("./picture/obstacle.png")
        # plt.show()
        plt.close(fig)
        
    return force, angle

def lasarField2(msgScan, odomMsg, xPoint, yPoint, plotFlag=False):
    """_summary_

    Args:
        msgScan (LaserScan): message get from robots' lidar 
        odomMsg (Odometry): message of the robot
        plotFlag (Bool) : whether to plot the field or not
        
    Returns:
        force: magnitude of the repulsive force
        angle: angle of the repulsive force
    """
    # Get robot position and orientation
    theta = getRotation(odomMsg)
    # Get lidar scan
    ( lidar, angles ) = lidarScan(msgScan)
    angles += rad_to_deg(theta)
    
    obs_x = []
    obs_y = []
    for i in range(len(lidar)):
        # if lidar[i] >= (MAX_LIDAR_DISTANCE - 0.0000001): continue
        # else: 
        obs_x.append(lidar[i] * math.cos(math.radians(angles[i])) + odomMsg.pose.pose.position.x)
        obs_y.append(lidar[i] * math.sin(math.radians(angles[i])) + odomMsg.pose.pose.position.y)
    # fig = plt.figure(figsize=(10, 10))
    # ax = fig.add_subplot(111)
    # ax.plot(obs_x, obs_y, 'o')
    
    # plt.savefig(packagePath+"/picture/obstacle points.png")
    # print(len(obs_x))
    
    min = 10.0
    obstacle_x = 0.0
    obstacle_y = 0.0
    for i in range(len(obs_x)):
        if getDistance(obs_x[i], xPoint, obs_y[i], yPoint) < min:
            min = getDistance(obs_x[i], xPoint, obs_y[i], yPoint)
            obstacle_x = obs_x[i]
            obstacle_y = obs_y[i]
            
    if min >= SAFE_DISTANCE: return 0, 0
    
    distToObs = min
    
    force = LASER_A * LASER_K * math.exp( - LASER_K * distToObs)
    angle = math.atan2( xPoint - obstacle_x, yPoint - obstacle_y )
        
    if (plotFlag):
        # Show field!!!!!
        x = np.linspace(-10, 10, 100) 
        y = np.linspace(-10, 10, 100)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros((100, 100))
        
        for i in range(100):
            for j in range(100):
                myDist = getDistance(X[i, j], obs_x, Y[i, j], obs_y)      
                Z[i, j] = LASER_A * math.exp(-LASER_K * myDist)
        
        fig = plt.figure(figsize=(10, 6))
        ax = fig.add_subplot(111, projection='3d')
        
        surf = ax.plot_surface(X, Y, Z, cmap="Blues_r", cstride=1, rstride=1)
        fig.colorbar(surf)
        plt.savefig("./picture/obstacle.png")
        # plt.show()
        plt.close(fig)
        
    return force, angle

def targetField(targetX, targetY, odomMsg, plotFlag=False):
    """_summary_

    Args:
        targetX (float): x coordinate of the target
        targetY (float): y coordinate of the target
        odomMsg (Odometry): message of the robot
        plotFlag (bool, optional): whether to plot the field or not. Defaults to False.

    Returns:
        force: magnitude of the attractive force
        angle: angle of the attractive force
    """
    ( myx , myy ) = getPosition(odomMsg)
    distToTarget = getDistance(myx, targetX, myy, targetY)
    if USE_TRADITION: force = 2 * TARGET_K * distToTarget
    else: force = TARGET_K 
    angle = math.atan2(targetY - myy, targetX - myx)
    
    if (plotFlag):
        x = np.linspace(-10, 10, 100)
        y = np.linspace(-10, 10, 100)

        X, Y = np.meshgrid(x, y)
        if USE_TRADITION: Z = TARGET_K * ((X-targetX)**2 + (Y-targetY)**2)
        else: Z = TARGET_K * ((X-targetX)**2 + (Y-targetY)**2)
        fig = plt.figure(figsize=(10, 6))
        ax = fig.add_subplot(111, projection='3d')
        
        surf = ax.plot_surface(X, Y, Z, cmap="Reds", cstride=1, rstride=1)
        fig.colorbar(surf)
        plt.savefig("./picture/target.png")
        # plt.show()
        plt.close(fig)
    
    return force, angle

def getDistance(x1, x2, y1, y2) :
    """Calculate the distance between two points."""
    return ( (x2 - x1)**2 + (y2 - y1)**2 ) ** 0.5

def evaluateLaser(msgScan, odomMsg1, x_goal, y_goal, xPoints, yPoints, plotFlag=False):
    """_summary_
    
    Args:
        msgScan (LaserScan): message get from robots' lidar 
        odomMsg1 (Odometry): message of the robot
        xPoints (list): list of x coordinates of points in the field that needed to sample 
        yPoints (list): list of y coordinates of points in the field that needed to sample 
        plotFlag (bool, optional): whether to plot the field or not. Defaults to False.
        
    Returns:
        list: list of values of the lasarField of each input point
    """
    
    Values = []
    nextXpoints = []
    nextYpoints = []
    useful_values = []
    for i in range(len(xPoints)):
        tempOdom = createOdom(xPoints[i], yPoints[i], odomMsg1.pose.pose.orientation)
        forceObs, angleObs = lasarField2(msgScan, odomMsg1, xPoints[i], yPoints[i])
        # print(forceObs, angleObs)
        forceTarget, angleTarget = targetField(x_goal, y_goal, tempOdom)
        forceX = forceObs * math.cos(angleObs) + forceTarget * math.cos(angleTarget)
        forceY = forceObs * math.sin(angleObs) + forceTarget * math.sin(angleTarget)
        nextXpoint = xPoints[i] + forceX*V_K
        nextXpoints.append(nextXpoint)
        nextYpoint = yPoints[i] + forceY*V_K
        nextYpoints.append(nextYpoint)
        if forceObs > DANGER_FORCE: Values.append(10000)
        else: 
            Values.append(getDistance(nextXpoint, x_goal, nextYpoint, y_goal))
            useful_values.append(getDistance(nextXpoint, x_goal, nextYpoint, y_goal))
    
    for enum, value in enumerate(Values):
        if value > 9000: continue
        else: 
            Values[enum] -= DISTANCE*5
            break
        
    # print(enum)
    # print(Values.index(min(Values)))
    
    if (plotFlag):
        print(Values)
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.scatter(xPoints, yPoints, Values)
        ax.set_zlim3d(0,20)
        plt.savefig(packagePath+"/picture/sample test.png")
        plt.close(fig)
        
        # Get robot position and orientation
        ( myx , myy ) = getPosition(odomMsg1)
        theta = getRotation(odomMsg1)
        # Get lidar scan
        ( lidar, angles ) = lidarScan(msgScan)
        angles += rad_to_deg(theta)
        obs_x = []
        obs_y = []
        for i in range(len(lidar)):
            obs_x.append(lidar[i] * math.cos(math.radians(angles[i])) + myx)
            obs_y.append(lidar[i] * math.sin(math.radians(angles[i])) + myy)
        fig = plt.figure()
        plt.plot(xPoints, yPoints, '.', color='r', label='generate points')
        plt.plot(nextXpoints, nextYpoints, '.', color='b', label='next points')
        plt.plot(obs_x, obs_y, '.', color='g', label='laser scan')
        # plt.legend(loc='best')  
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title("sample points and next points")
        plt.savefig(packagePath+'/picture/sample points and next points.png')
        plt.close(fig)
  
    return Values

def generatePoints(xPoint, yPoint, orientation, distance, num, plotFlag=False):
    """_summary_

    Args:
        xPoint (float): the xPoint of the center 
        yPoint (float): the yPoint of the center 
        distance (float): radius of the circle
        num (int): the number of the sample points
        plotFlag (bool, optional): whether to plot the field or not. Defaults to False.
        
    Returns:
        list: list of xPoints of sample points
        list: list of yPoints of sample points
    """
    x = xPoint
    y = yPoint
    xPoints = []
    yPoints = []
    for i in range(num):
        xPoints.append(x + distance*math.cos(2*math.pi/num*i+orientation))
        yPoints.append(y + distance*math.sin(2*math.pi/num*i+orientation))

    # xPoints.append(x)
    # yPoints.append(y)
    
    if (plotFlag):
        plt.plot(xPoints, yPoints, 'o')
        plt.savefig(packagePath+"/picture/sample points.png")
    
        # plt.show()
        plt.close()
        
    return xPoints, yPoints

def createOdom(xPoint, yPoint, quater):
    myOdom = Odometry()
    myOdom.pose.pose.position.x = xPoint
    myOdom.pose.pose.position.y = yPoint
    myOdom.pose.pose.orientation = quater
    return myOdom




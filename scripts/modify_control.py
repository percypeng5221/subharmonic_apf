#!/usr/bin/env python
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from math import atan2, cos, sin ,pi
import matplotlib.pyplot as plt

coefLinear = 1
coefAngular = 5
maxdx = 0.4
maxdw = 0.3
c = 0.01 #avoid too small velocity

def move(myOdometry, targetx, targety):
    """_summary_

    Args:
        myModelState (ModelState): modelState of the robot
        targetModelState (ModelState): target modelState of the robot, only x & y are given, no orientation

    Returns:
        Twist: command of the robot
    """
    
    myx = myOdometry.pose.pose.position.x
    myy = myOdometry.pose.pose.position.y
    # print(myOdometry.pose.pose.orientation)
    myRotation = getRotation(myOdometry)
    

    targetRotation = atan2(targety - myy , targetx - myx)
    
    dw = targetRotation - myRotation
    if(dw > pi/2):
        targetRotation = targetRotation + pi
    dw = targetRotation - myRotation
    dx = getDistance(myOdometry, targetx, targety)
    dx, dw = clip(dx, dw)

    
    command = Twist()
    
    command.angular.x = 0
    command.angular.y = 0
    command.angular.z = coefAngular * dw
    
    command.linear.x = coefLinear * (dx+c) 
    command.linear.y = 0
    command.linear.z = 0
    
    return command


def clip(dx, dw):
    if dx > maxdx: dx = maxdx
    elif dx < -1 * maxdx: dx= -1 * maxdx
    
    if dw > maxdw: dw = maxdw
    elif dw < -1 * maxdw: dw = -1 * maxdw
    return dx, dw 

def getRotation(myOdometry):
    """Calculate the rotation of the modelState"""
    orientation_q = myOdometry.pose.pose.orientation
    orientation_list = [ orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    return yaw

def getDistance(myOdometry,target_x,target_y):
    """Calculate the distance between two points."""
    myx = myOdometry.pose.pose.position.x
    myy = myOdometry.pose.pose.position.y
    return ( (myx - target_x)**2 + (myy - target_y) **2 ) ** 0.5


if __name__ == "__main__":

    dt = 0.01
    myOdometry = Odometry()

    
    myOdometry.pose.pose.position.x = 0
    myOdometry.pose.pose.position.y = 0
    myOdometry.pose.pose.orientation = quaternion_from_euler(0, 0, 0)

    target_x = 2
    target_y = -2
    
    x = []
    y = []
    
    while getDistance(myOdometry,target_x, target_y) > 1e-4:
        command = move(myOdometry, target_x, target_y)
        # print(command)
        prew = getRotation(myOdometry)
        x.append(myOdometry.pose.pose.position.x)
        y.append(myOdometry.pose.pose.position.y)
        print("X Y: ")
        print(myOdometry.pose.pose.position.x, myOdometry.pose.pose.position.y)
        print("Current rotation: ")
        print(prew)
        print("Command: ")
        print(command.linear.x, command.angular.z)
        myOdometry.pose.pose.position.x += dt * command.linear.x * cos(prew)
        myOdometry.pose.pose.position.y += dt * command.linear.x * sin(prew)
        neww = prew + command.angular.z * dt
        myOdometry.pose.pose.orientation = quaternion_from_euler(0, 0, neww)
        print("\n")
        
    # print(targetRotation)- myRotation
    plt.plot(x, y)
    plt.grid(True)
    plt.xlabel("X/m")
    plt.ylabel("Y/m")
    plt.title("Simulation of control algorithm")
    # plt.savefig("./simulation of control algorithm.png")
    plt.show()

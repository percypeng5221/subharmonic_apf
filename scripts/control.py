from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry
from math import atan2, cos, sin
import matplotlib.pyplot as plt

coefLinear = 1
coefAngular = 2
maxdx = 0.4
maxdw = 0.3

def move(myOdometry, targetx, targety):
    """_summary_

    Args:
        myOdometry (Odometry): Odometry of the robot
        targetx (float): x of target
        targety (float): y of target

    Returns:
        Twist: command of the robot
    """
    
    myx = myOdometry.pose.pose.position.x
    myy = myOdometry.pose.pose.position.y
    
    myRotation = getRotation(myOdometry)
    # print("position", targety, myy, targetx, myx)
    targetRotation = atan2(targety - myy , targetx - myx)
    
    dw = coefAngular * (targetRotation - myRotation)
    # print("rotation: ", targetRotation, myRotation)
    dx = coefLinear * getDistance(targetx, myx, targety, myy)
    
    dx, dw = clip(dx, dw)
    command = Twist()
    command.linear.x = dx 
    command.linear.y = 0
    command.linear.z = 0
    
    command.angular.x = 0
    command.angular.y = 0
    command.angular.z = dw
    
    return command

def getRotation(odometry):
    """Calculate the rotation of the odometry"""
    orientation_q = odometry.pose.pose.orientation
    orientation_list = [ orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    return yaw

def getDistance(x1, x2, y1, y2):
    """Calculate the distance between two points."""
    return ( (x2 - x1)**2 + (y2 - y1)**2 ) ** 0.5

def clip(dx, dw):
    if dx > maxdx: dx = maxdx
    elif dx < -1 * maxdx: dx= -1 * maxdx
    
    if dw > maxdw: dw = maxdw
    elif dw < -1 * maxdw: dw = -1 * maxdw
    return dx, dw   

if __name__ == "__main__":

    dt = 0.01
    myOdometry = Odometry()
    targetx = 2
    targety = 2
    
    myOdometry.pose.pose.position.x = 1
    myOdometry.pose.pose.position.y = 1
    # myOdometry.pose.pose.orientation = quaternion_from_euler(0, 0, 0)
    
    x = []
    y = []
    
    # while getDistance(targetx, myOdometry.pose.pose.position.x, 
    #                   targety, myOdometry.pose.pose.position.y) > 1e-4:
    for i in range(10000):
        command = move(myOdometry, targetx, targety)
        # print(command)
        prew = getRotation(myOdometry)
        x.append(myOdometry.pose.pose.position.x)
        y.append(myOdometry.pose.pose.position.y)
        # print("X Y: ")
        # print(myOdometry.pose.pose.position.x, myOdometry.pose.pose.position.y)
        # print("Current rotation: ")
        # print(prew)
        # print("Command: ")
        # print(command.linear.x, command.angular.z)
        myOdometry.pose.pose.position.x += dt * command.linear.x * cos(prew)
        myOdometry.pose.pose.position.y += dt * command.linear.x * sin(prew)
        neww = prew + command.angular.z * dt
        
        quaternion = quaternion_from_euler(0, 0, neww)
        myOdometry.pose.pose.orientation.x = quaternion[0]
        myOdometry.pose.pose.orientation.y = quaternion[1]
        myOdometry.pose.pose.orientation.z = quaternion[2]
        myOdometry.pose.pose.orientation.w = quaternion[3]
        
    plt.plot(x, y)
    plt.grid(True)
    plt.xlabel("X/m")
    plt.ylabel("Y/m")
    plt.title("Simulation of control algorithm")
    plt.savefig("./picture/simulation of control algorithm.png")
    # plt.show()
        
        
        
    
     

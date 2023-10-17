from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelState
from math import atan2, cos, sin
import matplotlib.pyplot as plt
import numpy as np
import sys
MODULES_PATH = '.'
sys.path.insert(0, MODULES_PATH)


a = [3, 4, 5, 2 ,1 ,6 ,7]
b = [2, 3, 4, 5, 6, 7, 8, 9, 10]

print(a.index(min(a)))

a = np.array([3, 4, 5, 2 ,1 ,6 ,7])
print(a.min())
### 1 Install ROS

Please follow the official instructions: https://wiki.ros.org/ROS/Installation

However, some convenient command line tools may be helpful:

​	https://wiki.ros.org/ROS/Installation/TwoLineInstall/ 

​	https://github.com/fishros/install

### 2 Set up turtlebot3 environment

Please follow the official instructions: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

3.1 Quick start guide - PC startup (noetic) and 6.1 Simulation - Gazebo simulation would be enough

After setting up the environment, put the launch and world file into the folders

```
catkin_make
source devel/setup.bash
roslaunch turtlebot
```



### 3 Clone this repository 

1. Create your own workspace, (e.g. test_ws) and create a src inside.

```
git clone
catkin_make
source devel/setup.bash
roslaunch subharmAPF move.launch
```


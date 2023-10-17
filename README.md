### 1 Install ROS

Please follow the official instructions: https://wiki.ros.org/ROS/Installation

However, some convenient command line tools may be helpful:

​	https://wiki.ros.org/ROS/Installation/TwoLineInstall/ 

​	https://github.com/fishros/install

### 2 Set up turtlebot3 environment

Please follow the official instructions: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

3.1 Quick start guide - PC startup (noetic) and 6.1 Simulation - Gazebo simulation would be enough

After setting up the environment, copy the files from the folder "turtlebot" of this repository to the turtlebot_simulation/turtlebot_gazebo. The files in the turtlebot/launch go into turtlebot_simulation/turtlebot_gazebo/launch and the files in the turtlebot/worlds go into turtlebot_simulation/turtlebot_gazebo/worlds.
Then go to the turtlebot workspace:

```
catkin_make
source devel/setup.bash
roslaunch turtlebot3_gazebo static_hard.launch 
roslaunch turtlebot3_gazebo static_easy.launch 
```



### 3 Build this repository 

1. Create your own workspace, (e.g. test_ws) and create a src inside. In the src folder, clone this repository.

```
git clone https://github.com/percypeng5221/subharmAPF.git
```

2. Compile and run this repository
```
catkin_make
source devel/setup.bash
roslaunch subharmAPF move.launch
```
3. Play with the parameters

Inside the subharmAPF folder, there's a config.yaml file. In this file there are a lot parameters to play with, including different path planning method(USE_TRADITION, USE_SAMPLE, ONLY_PLAN). 

You can use these parameters to reproduce the results in the paper.



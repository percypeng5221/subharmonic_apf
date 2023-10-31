cmake_minimum_required(VERSION 2.8.3)
project(subharmonic_apf)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs )

## Declare a catkin package
catkin_package()

# Install python scripts
catkin_install_python(PROGRAMS 
				scripts/control_node_1.py
				scripts/control.py
				scripts/Control.py
				scripts/field.py
				scripts/Lidar.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

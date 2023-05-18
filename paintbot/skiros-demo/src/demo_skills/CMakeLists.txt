cmake_minimum_required(VERSION 2.8.3)
project(demo_skills)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS demo_robot)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  skiros2_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS 
#  LIBRARIES 
 CATKIN_DEPENDS rospy skiros2_msgs
#  DEPENDS 
)

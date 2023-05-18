#  SkiROS2 demo

## Install

In an existing catkin workspace (note: you should be in the src folder of your catkin workspace):  

```
git clone https://github.com/Bjarne-AAU/skiros-demo.git
cd skiros-demo
./scripts/install-package.sh . $ROS_DISTRO
catkin build
```

Otherwise, you can create a new catkin workspace (note: this adds a new source in your .bashrc):  

```
git clone https://github.com/Bjarne-AAU/skiros-demo.git
cd skiros-demo
./scripts/install-workspace.sh . $ROS_DISTRO
```

## Launch

```
roslaunch demo_skills main.launch
```

## More info

Check out the wiki and the additional examples in skiros2_test_lib.  

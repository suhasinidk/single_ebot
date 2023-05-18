# Paintbot

## Installation and Building
1. Run `rosdep install paintbot_description` and `rosdep install paintbot_gazebo`. This should install all dependencies.
2. Run `catkin build` from within the `catkin_ws` directory.

## Running
1. Source the development `setup.sh` file (e.g. `source catkin_ws/devel/setup.sh`).
    * Note that this only needs to be done once at the beginning of a working session. Each separate terminal session requires this, however.
2. Run `roslaunch paintbot_gazebo paintbot.launch`.

## Robot
The robot used is based directly on the Kuka Youbot model available from the default model database in Gazebo.
The original YouBot SDF was converted to URDF using the [pysdf](http://wiki.ros.org/pysdf) ([GitHub](https://github.com/andreasBihlmaier/pysdf)) package.

#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/src/catkin_ws/src/roboquest_ui/src/RobotConsole/node_modules/opencv-build/opencv/build/lib

source /usr/src/catkin_ws/devel/setup.bash
roslaunch roboquest_ui server.launch

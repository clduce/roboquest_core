#!/usr/bin/env bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ubuntu/catkin_ws/src/roboquest_ui/src/RobotConsole/node_modules/opencv-build/opencv/build/lib
source ~/catkin_ws/devel/setup.bash
roslaunch roboquest_ui server.launch

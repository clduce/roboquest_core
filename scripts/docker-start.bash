#!/usr/bin/env /bin/bash

docker run -it --name roboquest --privileged --rm --net=host -v /var/run/dbus:/var/run/dbus  -v /dev/video0:/dev/video0  roboquest /bin/bash --login /home/ubuntu/catkin_ws/src/roboquest_core/scripts/start.bash

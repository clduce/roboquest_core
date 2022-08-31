#!/usr/bin/env /bin/bash

docker run --name roboquest --privileged --rm --net=host -v /var/run/dbus:/var/run/dbus  -v /dev/video0:/dev/video0  roboquest /bin/bash --login src/roboquest_core/scripts/start.bash

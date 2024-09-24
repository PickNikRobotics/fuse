#!/bin/bash

source /opt/ros/rolling/setup.bash

if [ -f /colcon_ws/install/local_setup.bash ]
then
    source /colcon_ws/install/local_setup.bash
fi

exec "$@"

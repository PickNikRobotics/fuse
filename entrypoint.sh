#!/bin/bash

source /opt/ros/humble/setup.bash

if [ -f /colcon_ws/install/local_setup.bash ]
then
    source /colcon_ws/install/local_setup.bash
fi

exec "$@"

#!/bin/bash

source /opt/ros/jazzy/setup.bash

export PATH="/robotrio/drone/bin:${PATH}"

export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/robotrio/drone/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/robotrio/drone/lib

exec /usr/bin/supervisord -c /etc/supervisor/conf.d/supervisord.conf
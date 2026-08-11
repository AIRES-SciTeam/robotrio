#!/bin/bash

export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/robotrio/drone/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/robotrio/drone/lib

export PX4_GZ_STANDALONE=1
export PX4_GZ_WORLD="scene"
export PX4_GZ_MODEL_POSE="0, 0, 0, 0, 0, 0"
export PX4_GZ_MODEL_NAME="x500"
export PX4_SIM_MODEL="gz_x500"
export PX4_SYS_AUTOSTART=4001 

exec px4 -i 1 "$@"
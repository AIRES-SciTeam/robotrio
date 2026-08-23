#!/bin/bash

export PX4_GZ_STANDALONE=1
export PX4_GZ_WORLD="scene"
export PX4_GZ_MODEL_NAME="x500"
export PX4_SIM_MODEL="gz_x500"
export PX4_SYS_AUTOSTART=4001 

exec px4 -w /PX4-Autopilot/ -i 1 "$@"


# export GZ_SIM_RESOURCE_PATH="/robotrio/scene:$GZ_SIM_RESOURCE_PATH"
# export GZ_SIM_RESOURCE_PATH="/robotrio/drone/models:$GZ_SIM_RESOURCE_PATH"

# export PX4_GZ_WORLD="scene"

# export PX4_SIM_MODEL="gz_x500"
# export PX4_GZ_MODEL_POSE="-5.0, 4.7, 0.01, 0, 0, 0"
# export PX4_SYS_AUTOSTART=4001

# exec px4 -w /PX4-Autopilot/ -i 1 "$@"

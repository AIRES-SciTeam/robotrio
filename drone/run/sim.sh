#!/bin/bash

cd /PX4-Autopilot

export PX4_GZ_STANDALONE=1
export PX4_GZ_WORLD="scene"
export PX4_GZ_MODEL_NAME="x500"
export PX4_SIM_MODEL="gz_x500"
export PX4_SYS_AUTOSTART=4001 

make px4_sitl gz_x500
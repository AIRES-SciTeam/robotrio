#!/bin/bash

cd /tmp

git clone --recursive https://github.com/PX4/PX4-Autopilot.git

cd PX4-Autopilot

make px4_sitl gz_x500 -j$(nproc)

mkdir -p /PX4-Autopilot

cp -r /tmp/PX4-Autopilot/build/px4_sitl_default/bin /PX4-Autopilot/

cp -r /tmp/PX4-Autopilot/build/px4_sitl_default/etc /PX4-Autopilot/

cp -r /tmp/PX4-Autopilot/build/px4_sitl_default/ROMFS /PX4-Autopilot/

cp -r /tmp/PX4-Autopilot/build/px4_sitl_default/rootfs /PX4-Autopilot/

rm -rf /tmp/PX4-Autopilot

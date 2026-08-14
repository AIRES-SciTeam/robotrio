#!/bin/bash

cd /tmp

git clone --recursive https://github.com/PX4/PX4-Autopilot.git

cd PX4-Autopilot

make px4_sitl -j$(nproc)

mkdir -p /robotrio/drone/bin

cp /tmp/PX4-Autopilot/build/px4_sitl_default/bin/px4 /robotrio/drone/bin/

chmod +x /robotrio/drone/bin/px4
    
rm -rf /tmp/PX4-Autopilot

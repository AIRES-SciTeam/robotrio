#!/bin/bash

source /opt/ros/jazzy/setup.bash

source /robotrio/drone/config/drone.conf

chmod +x /robotrio/drone/control/main.py

mkdir -p /robotrio/logs

if $RUN_WITHOUT_GZ -eq 1; then
  echo "🚀 Running without Gazebo"
  echo "Using minimal config (no Gazebo)"
  supervisord -c /robotrio/drone/config/supervisord-no-gz.conf
else
  echo "🚀 Running with Gazebo"
  echo "Using full config (with Gazebo)"
  supervisord -c /robotrio/drone/config/supervisord.conf
fi
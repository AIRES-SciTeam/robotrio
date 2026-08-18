#!/bin/bash

source /opt/ros/jazzy/setup.bash

mkdir -p /robotrio/drone/logs

exec /usr/bin/supervisord -c /robotrio/drone/config/supervisord.conf

#!/bin/bash

source /opt/ros/jazzy/setup.bash

exec /usr/bin/supervisord -c /robotrio/drone/supervisord.conf
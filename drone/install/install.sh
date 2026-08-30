#!/bin/bash

source /opt/ros/rolling/setup.bash

rm -rf /PX4-Autopilot
./PX4-Autopilot.sh

rm -rf /MicroXRCEAgent
./MicroXRCEAgent.sh

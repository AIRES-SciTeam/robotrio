#!/bin/bash

rm -rf /MicroXRCEAgent
./MicroXRCEAgent.sh

./wait_gz.sh

rm -rf /PX4-Autopilot
./PX4-Autopilot.sh

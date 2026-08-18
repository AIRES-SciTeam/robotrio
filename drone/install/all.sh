#!/bin/bash

chmod +x *.sh

rm -rf /MicroXRCEAgent
./MicroXRCEAgent.sh

rm -rf /PX4-Autopilot
./PX4-Autopilot.sh

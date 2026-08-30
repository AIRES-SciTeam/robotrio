#!/bin/bash

echo "Waiting for Gazebo"

MAX_RETRIES=30
RETRY=0


while [ $RETRY -lt $MAX_RETRIES ]; do
  if dpkg -l | grep -q "gz-transport" && \
    dpkg -l | grep -q "gz-sim" && \
    dpkg -l | grep -q "gz-sensors"; then
    echo "Gazebo is found!"
    
    ldconfig
      
    if pkg-config --exists gz-transport16 2>/dev/null || \
      find /usr/lib -name "gz-transport*Config.cmake" 2>/dev/null | grep -q .; then
      echo "Gazebo is found!"
      exit 0
    fi
  fi
  
  echo "Waiting... ($RETRY/$MAX_RETRIES)"
  sleep 2
  RETRY=$((RETRY+1))
done

echo "Gazebo is not found!"
exit 1

#!/bin/bash

source /opt/ros/rolling/setup.bash
source /robotrio/drone/drone.conf

mkdir -p /robotrio/logs

LOG_FILE="/robotrio/logs/startup.log"

{
    echo "=== Starting at $(date) ==="
    echo "Checking for world: $PX4_GZ_WORLD"
    
    # Проверяем, запущен ли Gazebo
    if gz service --list 2>&1 | grep -q "/world/$PX4_GZ_WORLD/"; then
        echo "✅ World '$PX4_GZ_WORLD' is running"
        echo "Using minimal config (no Gazebo)"
        supervisord -c /robotrio/drone/config/supervisord-no-gz.conf
    else
        echo "🚀 World '$PX4_GZ_WORLD' not found or Gazebo not running"
        echo "Using full config (with Gazebo)"
        supervisord -c /robotrio/drone/config/supervisord.conf
    fi
    
    echo "Starting PX4..."
} 2>&1 | tee -a "$LOG_FILE"

exec px4 -w /PX4-Autopilot/ -i 1 "$@"
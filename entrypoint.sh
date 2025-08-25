#!/usr/bin/env bash
set -e
# Source ROS 2
source /opt/ros/jazzy/setup.bash
# Source workspace overlay if it exists
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"

# robot-mBuff/run.sh
#!/usr/bin/env bash
set -e -o pipefail
source /opt/ros/jazzy/setup.bash

cd /ws
colcon build --symlink-install   # <— always build, fast for Python
source /ws/install/setup.bash

export WEB_BRIDGE_PORT="${WEB_BRIDGE_PORT:-5001}"
exec ros2 launch mbuff_bringup mbuff_min.launch.py

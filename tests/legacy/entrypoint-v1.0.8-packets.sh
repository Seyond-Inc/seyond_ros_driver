#!/usr/bin/env bash
set -e

# Source ROS2 Jazzy
source "/opt/ros/jazzy/setup.bash"

# Source the built workspace
if [ -f "/opt/metrolla/seyond/install/setup.bash" ]; then
    source "/opt/metrolla/seyond/install/setup.bash"
fi

# Execute the command passed from Docker CMD or Balena
exec "$@"
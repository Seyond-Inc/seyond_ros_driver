#!/usr/bin/env bash
set -e

# Source ROS2 Jazzy
source "/opt/ros/jazzy/setup.bash"

LIDAR_MODEL="${LIDAR_MODEL:-}"
LIDAR_MODEL_UPPER="$(echo "$LIDAR_MODEL" | tr '[:lower:]' '[:upper:]')"

if [[ "$LIDAR_MODEL_UPPER" == *"HUMMINGBIRD"* ]] || [[ "$LIDAR_MODEL_UPPER" == *"HB"* ]]; then
    echo "Detected Hummingbird model — using deb-based driver"
    # The deb installs the seyond package into /opt/ros/jazzy
    # No need to source a separate workspace
else
    echo "Using source-built driver"
    if [ -f "/opt/metrolla/seyond/install/setup.bash" ]; then
        source "/opt/metrolla/seyond/install/setup.bash"
    fi
fi

# Execute the command passed from Docker CMD or Balena
exec "$@"

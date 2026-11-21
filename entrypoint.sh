#!/usr/bin/env bash
# Metrolla driver contract, opt-in (AGENTS.md). Only when METROLLA_DRIVER_CONTRACT is set and not
# "0" does the image switch to the environment contract; otherwise everything below this block is
# the released entrypoint, byte for byte (tests/legacy pins it). Images without the contract (the
# -hb and -unified images) refuse the switch instead of ignoring it.
if [ -n "${METROLLA_DRIVER_CONTRACT:-}" ] && [ "${METROLLA_DRIVER_CONTRACT}" != "0" ]; then
    if [ -x /opt/metrolla/seyond/docker/contract-entrypoint.sh ]; then
        exec /opt/metrolla/seyond/docker/contract-entrypoint.sh "$@"
    fi
    echo "seyond-driver: refusing to start: METROLLA_DRIVER_CONTRACT=${METROLLA_DRIVER_CONTRACT} is set but this image has no environment contract (only the main seyond_drivers image does); accepted: unset or 0 here" >&2
    exit 1
fi
# --- end of the Metrolla contract block; the released entrypoint follows ---
set -e

# Source ROS2 Jazzy
source "/opt/ros/jazzy/setup.bash"

# Source the built workspace
if [ -f "/opt/metrolla/seyond/install/setup.bash" ]; then
    source "/opt/metrolla/seyond/install/setup.bash"
fi

# Execute the command passed from Docker CMD or Balena
exec "$@"
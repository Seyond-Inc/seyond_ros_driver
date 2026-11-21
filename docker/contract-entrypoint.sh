#!/usr/bin/env bash
# The Seyond driver image's environment contract (METROLLA_DRIVER_CONTRACT=1; AGENTS.md).
#
# entrypoint.sh execs this script when METROLLA_DRIVER_CONTRACT is set and not "0". It sources
# ROS 2 Jazzy and the driver workspace, then hands over to seyond_env.py, which checks the
# environment (refusing to start on any value it does not understand), writes the driver's
# configuration under /tmp and execs seyond_node with config_path pointing at it, so the node reads
# the rendered file and is the process that receives SIGTERM.
set -eo pipefail

# setup.bash reads unset variables; it must not run under `set -u`.
# shellcheck source=/dev/null
source "/opt/ros/jazzy/setup.bash"
# shellcheck source=/dev/null
source "/opt/metrolla/seyond/install/setup.bash"

exec python3 -I /opt/metrolla/seyond/docker/seyond_env.py "$@"

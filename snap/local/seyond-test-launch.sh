#!/bin/sh
# snap/local/seyond-test-launch.sh
#
# Runs the seyond_test diagnostic node (frame rate / packet loss checker)
# against the driver's topics. Extra arguments are passed through, e.g.:
#
#   ros2-seyond-lidar.seyond-test --ros-args -p topic_prefix:=/seyond_lidar -p hz:=true
set -eu

# `ros2 run` (ros2cli) isn't bundled; exec the node directly.
exec "$SNAP/opt/ros/snap/lib/seyond/seyond_test" "$@"

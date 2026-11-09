#!/bin/sh
# snap/local/seyond-launch.sh
#
# Entry point for the `seyond` daemon app. Resolves the driver config file
# and launches the seyond_node.
set -eu

# Managed default config seeded once by snap/hooks/install. Used when no
# `config-file` override is set.
DEFAULT_CONFIG_FILE="$SNAP_COMMON/config/seyond.yaml"
PARAMS_FILE="$DEFAULT_CONFIG_FILE"

# Separate location for URL-fetched overrides so the managed default is never
# overwritten.
CONFIG_FILE_DOWNLOAD="$SNAP_COMMON/config/seyond.remote.yaml"

CONFIG_FILE="$(snapctl get config-file)"

if [ -n "$CONFIG_FILE" ] && printf '%s' "$CONFIG_FILE" | grep -qE '^https?://'; then
  # Re-download on every start so upstream updates are picked up on restart.
  mkdir -p "$(dirname "$CONFIG_FILE_DOWNLOAD")"
  echo "seyond: downloading config-file override from $CONFIG_FILE" >&2
  if ! curl --fail --silent --show-error --location \
      --proto '=http,https' --proto-redir '=http,https' \
      --connect-timeout 10 --max-time 60 \
      --output "$CONFIG_FILE_DOWNLOAD.tmp" "$CONFIG_FILE"; then
    echo "seyond: failed to download config-file from $CONFIG_FILE" >&2
    rm -f "$CONFIG_FILE_DOWNLOAD.tmp"
    exit 1
  fi
  mv "$CONFIG_FILE_DOWNLOAD.tmp" "$CONFIG_FILE_DOWNLOAD"
  PARAMS_FILE="$CONFIG_FILE_DOWNLOAD"
elif [ -n "$CONFIG_FILE" ]; then
  PARAMS_FILE="$CONFIG_FILE"
fi

if [ ! -f "$PARAMS_FILE" ]; then
  echo "seyond: configuration file not found at $PARAMS_FILE" >&2
  echo "seyond: the snap seeds a default config on install; run '$SNAP_NAME.reset-config' or reinstall the snap" >&2
  exit 1
fi

# `ros2 run` (ros2cli) isn't bundled; exec the node directly.
set -- "$SNAP/opt/ros/snap/lib/seyond/seyond_node" --ros-args -p config_path:="$PARAMS_FILE"

echo "seyond: exec: $*" >&2
exec "$@"

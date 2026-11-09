#!/bin/sh
# snap/local/reset-config.sh
#
# Overwrites the managed config at $SNAP_COMMON/config/seyond.yaml with the
# packaged default, discarding any local edits, and unsets the `config-file`
# override so the managed default is used again.
set -eu

DEFAULT_CONFIG_FILE="$SNAP_COMMON/config/seyond.yaml"

mkdir -p "$SNAP_COMMON/config"
cp "$SNAP/etc/configuration/default-config.yaml" "$DEFAULT_CONFIG_FILE"
snapctl unset config-file

echo "seyond: reset $DEFAULT_CONFIG_FILE to packaged defaults (and unset 'config-file')" >&2
echo "seyond: run 'sudo snap restart $SNAP_NAME' to apply" >&2

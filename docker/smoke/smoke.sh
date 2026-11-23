#!/usr/bin/env bash
# Smoke tier of the Seyond driver image (TESTS.md). Needs Docker and python3.
# Usage: docker/smoke/smoke.sh IMAGE [OLD_IMAGE]
#
#   1. Image facts: entrypoint, command and user are what the fleets run today.
#   2. Legacy mode: for each fleet environment of tests/legacy/cases, the entrypoint hands the
#      command its environment unchanged and start_with_config.py picks the golden's YAML; with
#      OLD_IMAGE (the released image, e.g. seyond_drivers:v1.0.8-packets) both are compared with it.
#   3. Contract check and refusals: stdout/stderr byte-identical to tests/regression/golden.
#   4. Contract run as the product does it (uid 97000, read-only root, tmpfs /tmp, cap-drop ALL,
#      no-new-privileges, host network and IPC), no sensor: the node starts, keeps retrying
#      (continue_live) and stops cleanly.
#   5. Synthetic Robin-W: seyond_node (same restrictions) replays fake_seyond.py's packets from the
#      contract's configuration with only the file source added; PointCloud2 checked.
set -euo pipefail

IMAGE=${1:?usage: smoke.sh IMAGE [OLD_IMAGE]}
OLD_IMAGE=${2:-}
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORK=$(mktemp -d)
chmod 0755 "$WORK"
NAME=seyond-smoke-$$
FAILED=0

fail() { echo "SMOKE FAIL: $*" >&2; FAILED=1; }
ok() { echo "ok: $*"; }
# expect MESSAGE COMMAND...: ok when the command succeeds, a failure otherwise.
expect() { local msg=$1; shift; if "$@"; then ok "$msg"; else fail "$msg"; fi; }
cleanup() {
    docker rm -f "$NAME" "$NAME-replay" >/dev/null 2>&1 || true
    rm -rf "$WORK"
}
trap cleanup EXIT

env_args() {  # KEY=VALUE lines of a case file -> ENV_ARGS (skips comments and argv:)
    local line
    ENV_ARGS=()
    while IFS= read -r line; do
        [[ -z "$line" || "$line" == \#* || "$line" == argv:* ]] && continue
        ENV_ARGS+=(-e "$line")
    done < "$1"
}

# ---------------------------------------------------------------------------------------------
echo "== 1. image facts"
facts=$(docker image inspect "$IMAGE" --format '{{json .Config.Entrypoint}} {{json .Config.Cmd}} user={{.Config.User}}')
echo "$facts"
expect "entrypoint, command and user unchanged" \
    test "$facts" = '["entrypoint.sh"] ["ros2","launch","seyond","start_with_config.py"] user='

# ---------------------------------------------------------------------------------------------
echo "== 2. legacy mode, fleet environments"
# shellcheck disable=SC2016  # expanded by the shell inside the container
DUMP='echo "@@@ argv: $0 $*"; env | sort | grep -E "^(PATH|LD_LIBRARY_PATH|AMENT_PREFIX_PATH|ROS_DISTRO|ROS_VERSION|DUAL_LIDAR|ROS_LOCALHOST_ONLY|ROS_LOG_DIR|FASTRTPS_DEFAULT_PROFILES_FILE|USE_MACADDRESS_NS|TZ|METROLLA_DRIVER_CONTRACT)="'
legacy_dump() {  # IMAGE CASE
    env_args "$2"
    docker run --rm "${ENV_ARGS[@]}" "$1" bash -c "$DUMP" probe 2>&1
    if python3 -c 'import json,sys; sys.exit(0 if json.load(open(sys.argv[1]))["launch_config"] else 1)' \
        "$REPO/tests/legacy/golden/$(basename "$2" .env).json"; then
        docker run --rm "${ENV_ARGS[@]}" "$1" ros2 launch seyond start_with_config.py --show-args 2>&1 \
            | grep -A2 "'config_path'" || true
    fi
}
for case in "$REPO"/tests/legacy/cases/*.env; do
    stem=$(basename "$case" .env)
    legacy_dump "$IMAGE" "$case" > "$WORK/$stem.new"
    # The entrypoint prints nothing and passes the case's variables through unchanged.
    python3 - "$WORK/$stem.new" "$REPO/tests/legacy/golden/$stem.json" "$stem" <<'PY' || FAILED=1
import json, sys
out = open(sys.argv[1]).read()
golden = json.load(open(sys.argv[2]))
stem = sys.argv[3]
problems = []
head, _, rest = out.partition("@@@ argv: ")
if head != golden["stdout"] + golden["stderr"]:
    problems.append(f"entrypoint output {head!r}")
env = dict(line.split("=", 1) for line in rest.splitlines()[1:] if "=" in line and not line.startswith(" "))
for key, value in golden["exec"]["env"].items():
    if key != "PATH" and key in env and env[key] != value:
        problems.append(f"{key}={env[key]!r}, golden {value!r}")
    if key != "PATH" and key not in env and key in ("DUAL_LIDAR", "ROS_LOCALHOST_ONLY", "ROS_LOG_DIR"):
        problems.append(f"{key} missing in the command's environment")
if golden["launch_config"] and golden["launch_config"] not in out:
    problems.append(f"start_with_config.py does not default to {golden['launch_config']}")
for p in problems:
    print(f"SMOKE FAIL: legacy {stem}: {p}", file=sys.stderr)
if not problems:
    print(f"ok: legacy {stem}: entrypoint and launch configuration as the golden")
sys.exit(1 if problems else 0)
PY
    if [[ -n "$OLD_IMAGE" ]]; then
        legacy_dump "$OLD_IMAGE" "$case" > "$WORK/$stem.old"
        if [[ "$stem" == *contract* ]]; then
            ok "legacy $stem: compared with the golden only (the old image predates the switch)"
        elif diff -u "$WORK/$stem.old" "$WORK/$stem.new"; then
            ok "legacy $stem identical to $OLD_IMAGE"
        else
            fail "legacy $stem differs from $OLD_IMAGE (diff above)"
        fi
    fi
done

# ---------------------------------------------------------------------------------------------
echo "== 3. contract: check and refusals in the image equal the host-side goldens"
for case in "$REPO"/tests/regression/cases/*.env; do
    stem=$(basename "$case" .env)
    env_args "$case"
    set +e
    docker run --rm "${ENV_ARGS[@]}" "$IMAGE" check > "$WORK/$stem.out" 2> "$WORK/$stem.err"
    code=$?
    set -e
    {
        printf 'exit %s\n--- stdout\n' "$code"
        cat "$WORK/$stem.out"
        printf -- '--- stderr\n'
        cat "$WORK/$stem.err"
    } > "$WORK/$stem.golden"
    expect "contract $stem (exit $code)" diff -u "$REPO/tests/regression/golden/$stem.golden" "$WORK/$stem.golden"
done
set +e
docker run --rm -e METROLLA_DRIVER_CONTRACT=1 -e LIDAR_MODEL=robin-w -e SENSOR_IP=172.168.1.10 "$IMAGE" bash \
    > /dev/null 2> "$WORK/cmd.err"
code=$?
docker run --rm --read-only --user 97000:97000 -e METROLLA_DRIVER_CONTRACT=1 -e LIDAR_MODEL=robin-w \
    -e SENSOR_IP=172.168.1.10 "$IMAGE" > /dev/null 2> "$WORK/ro.err"
code_ro=$?
set -e
expect "unknown command refused" grep -q "unknown command" "$WORK/cmd.err"
expect "unknown command exits 1 ($code)" test "$code" = 1
expect "read-only root without a writable /tmp refused: $(cat "$WORK/ro.err")" grep -q "cannot write" "$WORK/ro.err"
expect "read-only root without a writable /tmp exits 1 ($code_ro)" test "$code_ro" = 1

# ---------------------------------------------------------------------------------------------
echo "== 4. contract run, no sensor"
RESTRICT=(--network host --ipc host --user 97000:97000 --read-only --tmpfs /tmp
          --cap-drop ALL --security-opt no-new-privileges:true -e ROS_LOCALHOST_ONLY=1
          -e ROS_HOME=/tmp/ros -e ROS_LOG_DIR=/tmp/ros/log)
CONTRACT_ENV=(-e METROLLA_DRIVER_CONTRACT=1 -e LIDAR_MODEL=seyond-robin-w -e SENSOR_IP=192.0.2.10
              -e SENSOR_ID=east -e DATA_PORT=18010 -e FRAME_ID=seyond_east)
docker run -d --name "$NAME" "${RESTRICT[@]}" "${CONTRACT_ENV[@]}" "$IMAGE" > /dev/null
sleep 15
expect "driver still running after 15 s without a sensor" \
    test "$(docker inspect -f '{{.State.Running}}' "$NAME")" = true
uid=$(docker exec "$NAME" stat -c %u /proc/1)
expect "driver runs as uid $uid (97000)" test "$uid" = 97000
comm=$(docker exec "$NAME" cat /proc/1/comm)
expect "PID 1 is the driver ($comm)" test "$comm" = seyond_node
docker run --rm "${CONTRACT_ENV[@]}" "$IMAGE" check | sed '$d' > "$WORK/expected.yaml"
docker exec "$NAME" cat /tmp/seyond/config.yaml > "$WORK/actual.yaml"
expect "/tmp/seyond/config.yaml in the container equals check's render" \
    diff -u "$WORK/expected.yaml" "$WORK/actual.yaml"
# The observer needs its own DDS discovery time; sleeping before creating it does not help.
# Keep one participant alive while polling, and retain the exact node-name assertion.
if docker run --rm "${RESTRICT[@]}" -v "$HERE:/smoke:ro" --entrypoint bash "$IMAGE" -c \
    'source /opt/ros/jazzy/setup.bash && timeout --kill-after=5s 65s python3 /smoke/check_node.py /east/seyond --timeout 60' \
    > "$WORK/nodes.txt" 2>&1; then
    cat "$WORK/nodes.txt"
    expect "node /east/seyond in the ROS graph" grep -qx /east/seyond "$WORK/nodes.txt"
else
    cat "$WORK/nodes.txt"
    fail "node /east/seyond in the ROS graph: discovery check failed (60 s deadline; 65 s process limit)"
fi
docker stop -t 10 "$NAME" > /dev/null
code=$(docker inspect -f '{{.State.ExitCode}}' "$NAME")
docker logs "$NAME" > "$WORK/driver.log" 2>&1
grep -E "seyond-driver:|inno_lidar_start|continue_live" "$WORK/driver.log" | head -8
expect "the contract's start line is logged" grep -q "seyond-driver: Seyond Robin-W at 192.0.2.10" "$WORK/driver.log"
expect "stopped by SIGTERM, not SIGKILL (exit $code)" test "$code" != 137

# ---------------------------------------------------------------------------------------------
echo "== 5. synthetic Robin-W replayed through seyond_node"
python3 "$HERE/fake_seyond.py" "$WORK/fake.inno_pc" --frames 100
chmod 0644 "$WORK/fake.inno_pc"
REPLAY_ENV=(-e METROLLA_DRIVER_CONTRACT=1 -e LIDAR_MODEL=robin-w -e SENSOR_IP=172.168.1.10
            -e SENSOR_ID=west -e FRAME_ID=seyond_west)
# The contract's configuration with only the source switched to the file (looping).
docker run --rm "${REPLAY_ENV[@]}" "$IMAGE" check | sed '$d' \
    | sed -e 's|inno_pc_file: ""|inno_pc_file: "/data/fake.inno_pc"|' -e 's|file_rewind: 0|file_rewind: -1|' \
    > "$WORK/replay.yaml"
grep -q 'inno_pc_file: "/data/fake.inno_pc"' "$WORK/replay.yaml"
chmod 0644 "$WORK/replay.yaml"
docker run -d --name "$NAME-replay" "${RESTRICT[@]}" -v "$WORK:/data:ro" --entrypoint bash "$IMAGE" -c \
    'source /opt/ros/jazzy/setup.bash && source /opt/metrolla/seyond/install/setup.bash && exec /opt/metrolla/seyond/install/seyond/lib/seyond/seyond_node --ros-args -r __ns:=/west -p config_path:=/data/replay.yaml' \
    > /dev/null
set +e
docker run --rm "${RESTRICT[@]}" -v "$HERE:/smoke:ro" --entrypoint bash "$IMAGE" -c \
    'source /opt/ros/jazzy/setup.bash && python3 /smoke/check_cloud.py /west/iv_points seyond_west --clouds 3 --timeout 60' \
    > "$WORK/check.json"
code=$?
set -e
cat "$WORK/check.json"
expect "3 PointCloud2 on /west/iv_points decoded from synthetic packets (exit $code)" test "$code" = 0
docker logs "$NAME-replay" 2>&1 | grep -E "get first frame|play rate|corrup|crc" | head -5 || true
docker rm -f "$NAME-replay" > /dev/null

if [[ $FAILED != 0 ]]; then
    echo "== contract driver log"; cat "$WORK/driver.log"
    echo "SMOKE FAILED"
    exit 1
fi
echo "SMOKE PASSED"

# AGENTS.md — seyond_ros_driver

## What this repo is

The **Seyond (Innovusion) lidar ROS 2 driver**, packaged into Docker images
for Metrolla edge units:

- `src/seyond_lidar_ros/` — the vendor driver source (ROS 1 + ROS 2
  adapters, ROS 2 Jazzy is what we ship). `seyond_node` connects to the
  lidar over TCP/UDP (`lidar_ip`, default `172.168.1.10`) and publishes the
  pointcloud on `frame_topic` (default `/iv_points`).
- `src/seyond_lidar_ros/launch/start_with_config.py` — the launch entry the
  images use; it selects the YAML config that drives the node.
- `src/seyond_lidar_ros/config/config.yaml` — single-lidar config.
- `src/seyond_lidar_ros/config/two-config.yaml` — dual-lidar config
  (lidar-2 on `/iv_points2`). **Exists only from v1.0.7-dual /
  `feat/dualdriver`** — not on `main` yet.
- `Dockerfile` / `Dockerfile.deb` / `Dockerfile.unified` + entrypoints —
  image builds; the vendored SDK ships as
  `seyond-ros2-jazzy-*.deb` (+ a git submodule), which is why CI has **no
  colcon build job** (Phase-5 work; see TESTS.md).

## Single vs dual images, fleet usage

- Version tags: `v1.0.x` = single-lidar; `v1.0.x-dual` = dual-lidar image.
- Edge fleets (self-hosted **openBalena** at `ob.metrolla.com`, repo
  `balena-fleets`) pin the image via the **`DRIVER_VERSION`** fleet/device
  variable — bumping `DRIVER_VERSION` is how a unit moves driver versions.
- Dual behavior at runtime is selected by the **`DUAL_LIDAR`** env var on
  the device (`1`/`true` ⇒ `two-config.yaml`; unset/`0`/`""`/`false` ⇒
  `config.yaml`).
- History: `v1.0.5-dual`/`v1.0.6-dual` were broken in the field (crash-loop
  at niagra-bottling precise-doorway, stopgap patched in-container);
  `v1.0.7-dual` (commits `4a4e5f2`/`e7f10f0`, branch `feat/dualdriver`) is
  the repo fix. Registry blobs live in Cloudflare R2 — pushes are capped by
  a ~40 Mbps uplink; never run parallel/duplicate pushes.

## Driver settings: legacy mode and the opt-in contract (C9)

The main image (`Dockerfile`, `ghcr.io/metrolla-inc/seyond_drivers:<tag>`) runs in one of two
modes, chosen per container by **`METROLLA_DRIVER_CONTRACT`**:

| `METROLLA_DRIVER_CONTRACT` | Mode | Who |
|---|---|---|
| unset, empty or `0` | **legacy**: exactly the released image (v1.0.8-packets) | every existing fleet |
| `1` | **contract**: the environment contract below | metrolla-edge, new deployments |
| anything else | refused: exit 1, one line naming the variable and `1` | — |

`entrypoint.sh` (and `entrypoint-unified.sh`) start with a block that, only when the switch is set
and not `0`, execs `/opt/metrolla/seyond/docker/contract-entrypoint.sh`; the rest is the released
entrypoint byte for byte (`tests/legacy`). The `-hb` and `-unified` images have no contract: the
block refuses the switch there (exit 1) instead of ignoring it. Image entrypoint, default command
and user (root) are unchanged.

**Legacy mode** is unchanged: the command (image CMD `ros2 launch seyond start_with_config.py`,
or the fleets' `bash -lc "... ros2 launch seyond start.py lidar_ip:=..."`) runs with the
environment as given; `DUAL_LIDAR` selects `two-config.yaml` in `start_with_config.py`.

**Contract mode** follows the contract of the LSLiDAR, Quanergy and Hesai images
(`Metrolla-Inc/lslidar_ros2_driver`, `Metrolla-Inc/quanergy_client_ros`,
`Metrolla-Inc/HesaiLidar_ROS_2.0`). `docker/seyond_env.py` checks the environment, renders the
vendor-format `config.yaml` for one lidar to `$SEYOND_RUN_DIR/config.yaml` (`/tmp/seyond`), and
execs `seyond_node --ros-args [-r __ns:=/<SENSOR_ID>] -p config_path:=<that file>` (PID 1, gets
SIGTERM). Subcommands: none or `run` (the image CMD means `run` too); `check` / `check --json`.
Anything else is refused. Refusal = exit 1, one stderr line per problem naming the variable and
what it accepts.

| Variable | Required | Accepted | Default | Becomes |
|---|---|---|---|---|
| `METROLLA_DRIVER_CONTRACT` | yes | `1` | — | contract mode |
| `LIDAR_MODEL` | yes | `robin-w`, `robin-elite` (`robin-e1x`), `robin-e2`, `falcon-k1`, `falcon-k2`, `falcon-k24`; any case, optional `seyond-`, `-`/`_` ignored. `hb-d1`/`hummingbird` refused (-hb image), `robin`/`falcon` refused (ambiguous) | — | a check and the file header; the SDK identifies the sensor itself |
| `SENSOR_IP` | yes | the sensor's IPv4 address, unicast | — | `lidar_ip` (the SDK drops UDP from other sources) |
| `SENSOR_ID` | no | `^[a-z][a-z0-9_]{0,30}$` without `__` | none | namespace `/<id>` and `lidar_name` |
| `DATA_PORT` | no | 1024-65535 | 8010 | `udp_port` (always UDP data) |
| `DEVICE_PORT` | no | 1024-65535 | 8010 | `port` (the sensor's TCP command port; equal to `DATA_PORT` is fine) |
| `HOST_IP` | no | IPv4, optionally `/1`-`/32`; not the sensor; with a prefix the sensor must be inside | none | a check only |
| `FRAME_ID` | no | 1-64 of letters, digits, `_`, not starting with a digit | `seyond` | `frame_id` |
| `POINTCLOUD_TOPIC` | no | a ROS topic name | `iv_points` (relative: `/iv_points` without `SENSOR_ID`) | `frame_topic` |
| `TIME_MODE` | no | `sensor`, `gps`, `ptp`, `ntp`; **`host` is refused**: the driver always stamps with the sensor's clock | `sensor` | the file header only |
| `PUBLISH_SCAN` | no | false values only | — | the driver has no LaserScan |
| `SEYOND_RUN_DIR` | no | an absolute path | `/tmp/seyond` | where `config.yaml` is written |
| `DUAL_LIDAR` | no | unset, empty or false | — | one container per sensor (each with its own `DATA_PORT`) |

- Fixed in every render: `continue_live: true` (gotcha 4), UDP data, no file/pcap source,
  `packet_topic: iv_packets` (relative); every other key is the vendor `config.yaml` value.
- Empty values refused; typos and other drivers' names refused with "did you mean" (`LIDAR_IPS` ->
  `SENSOR_IP`, `UDP_PORT` -> `DATA_PORT`, ...) for names starting `SEYOND_`, `LIDAR_`, `SENSOR_`,
  `DATA_`, `DEVICE_`, `HOST_`, `TIME_`, `FRAME_`, `POINTCLOUD_`, `POINT_`, `PUBLISH_`, `UDP_`,
  `DUAL_`, `CONTINUE_`. `METROLLA_*`, `ROS_*`, `RMW_*`, `FASTRTPS_*`, `HOME`, `TZ` pass through.
- Any uid (97000 in the product), read-only root with a tmpfs `/tmp` (else "cannot write"), host
  network and IPC, `cap_drop: [ALL]`, `no-new-privileges`; no privileges or `NET_ADMIN` needed. No
  health check is built in (it would change legacy containers); compose can use
  `grep -qsx seyond_node /proc/[0-9]*/comm` (liveness only).
- Without a sensor the node stays up and retries `inno_lidar_start` every 10 s (continue_live).
- `seyond_node` exits with status **0** when its YAML does not parse (vendor code): a
  `restart: on-failure` policy would not restart it. The contract's render always parses.

metrolla-edge's driver template would pass (instead of today's bind-mounted `config.yaml` and
`command`):

```yaml
    user: {{ ids.dds }}
    read_only: true
    tmpfs: [/tmp]
    environment:
      METROLLA_DRIVER_CONTRACT: 1
      SENSOR_ID: {{ sensor.id }}
      LIDAR_MODEL: {{ sensor.model }}        # seyond-robin-w
      SENSOR_IP: {{ sensor.ip }}
      DATA_PORT: {{ sensor.data_port }}
      FRAME_ID: seyond
      POINTCLOUD_TOPIC: /iv_points           # or the default /<sensor.id>/iv_points
      TIME_MODE: ptp
      ROS_LOCALHOST_ONLY: 1
```

Base images: `ros:jazzy-perception-noble` as before, pinned in all three Dockerfiles by its index
digest `sha256:233408de3a53d864d7f5e1c8c6ff7948d126162f22221c8f0a8785ab7b03095b` (Docker Hub,
2026-11-21). **Releases**: the three image workflows push only for a pushed `v*` git tag
(`seyond_drivers:<tag>`, `<tag>-hb`, `<tag>-unified`), never overwrite an existing image tag, and
record the digest; pull requests build without pushing; branch pushes push nothing (before: `main`,
`master` and `v*` branches pushed `:<branch>`, `-hb`, `-unified`, and every build rewrote the cache
tags `:latest`, `:latest-hb`, `:latest-unified`).

## Gotchas (the bug classes this gate exists for)

1. **Always-true env checks.** `os.environ.get('X') is True or 1 or "1"`
   parses as `(x is True) or 1 or "1"` and is constant-true; and
   `environ.get(...) is True` alone is constant-FALSE (env values are
   strings). Either way the flag is not actually consulted. Parse env flags
   with an explicit truthy-string set (`{'1','true','yes','on'}`), and keep
   a source-level test asserting the broken pattern stays absent.
2. **Config files must be CMake-installed.** A config referenced by a
   launch file but missing from the `install(FILES … DESTINATION share/…)`
   block exists in the repo yet not in the image ⇒ crash-loop only at
   deploy time. Every new config needs an install rule AND a test.
3. **Topic-name typos are silent.** `/iv_point2` vs `/iv_points2` produces
   zero errors — data just never reaches downstream. Topic names are a
   contract with consumers; pin them in tests, don't eyeball them.
4. **`continue_live: false` = zombie driver.** The start-retry loop in
   `driver_lidar.cc` is gated on `continue_live`. False means one connection
   attempt ever; a lidar that is late to boot leaves a running-but-deaf node
   that connectivity checks call healthy. Ship `true` on edges. Since the
   2026-08-31 vendor sync (upstream `18c5c93`) there is no fatal-error
   reconnect any more: mid-run recovery is the SDK's UDP read loop, so ship
   UDP data (`udp_port >= 0`), never TCP data.

## Setup / running tests

CI uses Python 3.12; tests need only `pytest` and `pyyaml` (and the SDK submodule:
`git submodule update --init`):

```bash
python3 -m pip install pytest pyyaml
python3 -m pytest tests/ -v -ra
```

See **TESTS.md** for the full suite inventory, expected counts on `main`
vs `feat/dualdriver`, the strict-xfail policy, and the PASS/FAIL/SKIP/ABORT
outcome semantics.

## Rules for agents working here

- The regression-gate program is **strictly additive**: add tests/CI/docs;
  do not modify driver source or delete files. (C9 changed no driver source; it replaced the
  stale fatal-error-reconnect assertion in `tests/test_continue_live.py`, see TESTS.md.)
- Legacy mode must not change: `tests/legacy` compares fleet environments with goldens made
  from the released entrypoint. A contract change changes a golden: `REGEN=1`, read the diff,
  update the table above.
- Fix-presence tests are **conditional strict xfails** keyed off the tree
  itself. When `feat/dualdriver` merges to `main` they flip to required-PASS
  automatically; if one then errors as XPASS-strict, delete its marker —
  never loosen the assertion.
- Verify, don't guess: never report a suite that could not run as healthy
  (ABORT ≠ PASS).

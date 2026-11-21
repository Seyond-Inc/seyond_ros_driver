#!/usr/bin/env python3
"""Environment contract of the Metrolla Seyond driver image (opt-in: METROLLA_DRIVER_CONTRACT=1).

Reads the container's environment, refuses to start on any value it does not understand, renders
the driver's YAML configuration (the vendor format of src/seyond_lidar_ros/config/config.yaml, one
lidar) and (subcommand ``run``) execs ``seyond_node`` with the ROS parameter ``config_path``
pointing at the rendered file, so the node reads exactly that file and is the process that receives
SIGTERM.

Standard library only: it runs in the image before the driver and in the unit and regression tests
outside the image, where there is no ROS.

Subcommands:
  run            validate, write the configuration, exec the driver (the default)
  check [--json] validate and print what ``run`` would write and exec; nothing is started
  selfcheck      for the image build: a configuration renders, the driver binary exists

Exit codes: 0 ok; 1 refused (a value this image does not understand, nowhere to write, or an
unknown command).
"""

from __future__ import annotations

import difflib
import ipaddress
import json
import os
import re
import sys
from dataclasses import dataclass, field
from typing import Mapping

PROG = "seyond-driver"
EXIT_OK = 0
EXIT_REFUSED = 1
EXIT_USAGE = 1

# Paths inside the image (Dockerfile: WORKDIR /opt/metrolla/seyond, COPY . ., build.bash = colcon
# build without --merge-install, so the package installs under install/seyond).
WORKSPACE = "/opt/metrolla/seyond"
DRIVER_NODE = WORKSPACE + "/install/seyond/lib/seyond/seyond_node"
NODE_NAME = "seyond"  # rclcpp::Node::make_shared("seyond", ...) in ros2_driver_adapter.hpp
DEFAULT_RUN_DIR = "/tmp/seyond"
DEFAULT_ROS_HOME = "/tmp/ros"

DEFAULT_DATA_PORT = 8010  # vendor config.yaml udp_port
DEFAULT_DEVICE_PORT = 8010  # vendor config.yaml port (the sensor's TCP command port)
MIN_PORT = 1024
MAX_PORT = 65535
DEFAULT_FRAME_ID = "seyond"  # vendor config.yaml frame_id
# Relative on purpose: without SENSOR_ID it resolves to /iv_points, the vendor value the stack
# reads; with SENSOR_ID it follows the node's namespace, so two sensors never share a topic.
DEFAULT_TOPIC = "iv_points"

TRUE_WORDS = ("1", "true", "yes", "on")
FALSE_WORDS = ("0", "false", "no", "off")
# The Seyond driver stamps clouds with the sensor's own clock (ts_start_us of the packets); it has
# no host-time option, so "host" is refused rather than silently ignored.
TIME_MODES = ("sensor", "gps", "ptp", "ntp")
DEFAULT_TIME_MODE = "sensor"

CONTRACT_SWITCH = "METROLLA_DRIVER_CONTRACT"
CONTRACT_VERSIONS = ("1",)


# --------------------------------------------------------------------------------------------
# Models: the sensors the SDK's README lists (seyond_sdk/README.md "Supported Lidar"), except the
# HB-D1, which this image's driver does not run (the -hb / -unified images use the vendor's deb).
# Nothing in the rendered configuration depends on the model: the SDK identifies the sensor from
# its packets. The model is checked so that a wrong deployment fails loudly.
# --------------------------------------------------------------------------------------------

MODELS = {"robinw": "Robin-W", "robinelite": "Robin-ELITE", "robine1x": "Robin-ELITE",
          "robine2": "Robin-E2", "falconk1": "Falcon-K1", "falconk2": "Falcon-K2",
          "falconk24": "Falcon-K24"}
REFUSED_MODELS = {
    "hbd1": "is a Hummingbird, which this image's driver does not run; use the -hb image (legacy "
            "mode only)",
    "hummingbird": "is a Hummingbird, which this image's driver does not run; use the -hb image "
                   "(legacy mode only)",
    "hb": "is a Hummingbird, which this image's driver does not run; use the -hb image (legacy "
          "mode only)",
    "robin": "is ambiguous: robin-w, robin-elite or robin-e2",
    "falcon": "is ambiguous: falcon-k1, falcon-k2 or falcon-k24",
}


def normalise_model(value: str) -> str:
    """Lowercase, without an optional 'seyond-' prefix, '-' and '_' removed. Anything else (a space,
    a dot) is kept, so it cannot match a model and the name is refused."""
    key = value.lower()
    if key.startswith("seyond-") or key.startswith("seyond_"):
        key = key[len("seyond-"):]
    return key.replace("-", "").replace("_", "")


def model_list() -> str:
    return "robin-w, robin-elite (robin-e1x), robin-e2, falcon-k1, falcon-k2, falcon-k24"


# --------------------------------------------------------------------------------------------
# Environment variables.
# --------------------------------------------------------------------------------------------

REQUIRED = ("LIDAR_MODEL", "SENSOR_IP")
OPTIONAL = ("SENSOR_ID", "DATA_PORT", "DEVICE_PORT", "HOST_IP", "FRAME_ID", "POINTCLOUD_TOPIC",
            "TIME_MODE", "PUBLISH_SCAN", "SEYOND_RUN_DIR")
COMPAT = ("DUAL_LIDAR",)  # this image's legacy dual flag: only false in contract mode
KNOWN = REQUIRED + OPTIONAL + COMPAT
ALIASES = {"LIDAR_IP": "SENSOR_IP", "LIDAR_IPS": "SENSOR_IP", "DEVICE_IP": "SENSOR_IP",
           "LIDAR_TYPE": "LIDAR_MODEL", "SENSOR_MODEL": "LIDAR_MODEL", "FRAME_TOPIC": "POINTCLOUD_TOPIC",
           "TIME_SERVICE": "TIME_MODE", "HOST_ADDRESS": "HOST_IP", "UDP_PORT": "DATA_PORT",
           "LIDAR_PORT": "DEVICE_PORT", "SENSOR_PORT": "DATA_PORT", "LIDAR_NAME": "SENSOR_ID",
           "POINT_CLOUD_TOPIC": "POINTCLOUD_TOPIC"}
STRICT_PREFIXES = ("SEYOND_", "LIDAR_", "SENSOR_", "DATA_", "DEVICE_", "HOST_", "TIME_", "FRAME_",
                   "POINTCLOUD_", "POINT_", "PUBLISH_", "UDP_", "DUAL_", "CONTINUE_")

ACCEPTED = {
    "LIDAR_MODEL": f"a Seyond model: {model_list()}",
    "SENSOR_IP": "the sensor's own IPv4 address, for example 172.168.1.10",
    "SENSOR_ID": "a lowercase letter then up to 30 lowercase letters, digits or single '_'",
    "DATA_PORT": f"a UDP port {MIN_PORT}-{MAX_PORT} (default {DEFAULT_DATA_PORT})",
    "DEVICE_PORT": f"the sensor's TCP command port {MIN_PORT}-{MAX_PORT} (default {DEFAULT_DEVICE_PORT})",
    "HOST_IP": "the host's IPv4 address on the sensor network, optionally with /1-/32",
    "FRAME_ID": f"letters, digits and '_', not starting with a digit (default {DEFAULT_FRAME_ID})",
    "POINTCLOUD_TOPIC": f"a ROS topic name (default {DEFAULT_TOPIC})",
    "TIME_MODE": f"sensor, gps, ptp or ntp (default {DEFAULT_TIME_MODE})",
    "PUBLISH_SCAN": "0, false, no or off (the Seyond driver publishes no LaserScan)",
    "SEYOND_RUN_DIR": f"an absolute directory path (default {DEFAULT_RUN_DIR})",
}

_PORT_RE = re.compile(r"[1-9][0-9]{0,4}")
_OCTET = r"(?:25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9]?[0-9])"
_IPV4_RE = re.compile(rf"{_OCTET}\.{_OCTET}\.{_OCTET}\.{_OCTET}")
_PREFIX_RE = re.compile(r"[1-9]|[12][0-9]|3[0-2]")
_SENSOR_ID_RE = re.compile(r"[a-z][a-z0-9_]{0,30}")  # metrolla-edge deployment schema, sensors[].id
_FRAME_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_]{0,63}")
_TOPIC_RE = re.compile(r"/?[A-Za-z][A-Za-z0-9_]*(?:/[A-Za-z][A-Za-z0-9_]*)*")
_RUN_DIR_RE = re.compile(r"/[A-Za-z0-9_./-]*")


@dataclass
class Settings:
    model: str
    sensor_ip: str
    sensor_id: str | None
    data_port: int
    device_port: int
    host_ip: str | None
    frame_id: str
    topic: str
    time_mode: str
    run_dir: str
    sources: dict[str, str] = field(default_factory=dict)

    @property
    def namespace(self) -> str:
        return "/" + self.sensor_id if self.sensor_id else "/"

    @property
    def node_fqn(self) -> str:
        return (self.namespace + "/" + NODE_NAME) if self.sensor_id else "/" + NODE_NAME

    @property
    def config_path(self) -> str:
        return self.run_dir.rstrip("/") + "/config.yaml"


class Refused(Exception):
    def __init__(self, problems: list[str]):
        super().__init__("\n".join(problems))
        self.problems = problems


def shown(value: str) -> str:
    text = value if len(value) <= 64 else value[:61] + "..."
    if text.isprintable() and text.isascii():
        return "'" + text + "'"
    return ascii(text)


def _ipv4(name: str, value: str, problems: list[str], what: str) -> ipaddress.IPv4Address | None:
    if not _IPV4_RE.fullmatch(value):
        problems.append(f"{name}={shown(value)} is not an IPv4 address; accepted: {what} in dotted "
                        "decimal without leading zeros, for example 172.168.1.10")
        return None
    address = ipaddress.IPv4Address(value)
    if address.is_unspecified or address.is_multicast or address.is_reserved or \
            address == ipaddress.IPv4Address("255.255.255.255"):
        problems.append(f"{name}={shown(value)} is not a unicast address; accepted: {what}, not "
                        "0.0.0.0, multicast (224-239.x.x.x), 240.x.x.x and above or broadcast")
        return None
    return address


def _port(name: str, value: str, problems: list[str], default_text: str) -> int | None:
    if not _PORT_RE.fullmatch(value) or not MIN_PORT <= int(value) <= MAX_PORT:
        problems.append(f"{name}={shown(value)} is not a usable port; accepted: a whole number "
                        f"{MIN_PORT}-{MAX_PORT} without leading zeros ({default_text})")
        return None
    return int(value)


def _bool(name: str, value: str, problems: list[str], default_text: str) -> bool | None:
    word = value.lower()
    if word in TRUE_WORDS:
        return True
    if word in FALSE_WORDS:
        return False
    problems.append(f"{name}={shown(value)} is not a boolean; accepted: {', '.join(TRUE_WORDS)} or "
                    f"{', '.join(FALSE_WORDS)}, in any case ({default_text})")
    return None


def parse(env: Mapping[str, str]) -> Settings:
    """Validate the environment. Raises Refused with one line per problem."""
    problems: list[str] = []
    sources: dict[str, str] = {}

    def get(name: str) -> str | None:
        value = env.get(name)
        sources[name] = "default" if value is None else "env"
        return value

    switch = env.get(CONTRACT_SWITCH)
    if switch not in CONTRACT_VERSIONS:
        problems.append(f"{CONTRACT_SWITCH}={shown(switch or '')} is not a contract version this "
                        f"image knows; accepted: {', '.join(CONTRACT_VERSIONS)} (or unset or 0 for "
                        "the legacy entrypoint)")

    for name in REQUIRED + OPTIONAL:  # an empty DUAL_LIDAR means false, as in legacy mode
        if name in env and env[name] == "":
            problems.append(f"{name} is set but empty; accepted: {ACCEPTED[name]}"
                            + ("" if name in REQUIRED else ", or unset it for the default"))
    present = {k: v for k, v in env.items() if k in KNOWN and v != ""}

    # LIDAR_MODEL
    model = None
    if "LIDAR_MODEL" not in env:
        problems.append("LIDAR_MODEL is not set; accepted (any case, optional 'seyond-' prefix, '-' "
                        f"and '_' ignored): {model_list()}")
    elif "LIDAR_MODEL" in present:
        raw = present["LIDAR_MODEL"]
        key = normalise_model(raw)
        if key in REFUSED_MODELS:
            problems.append(f"LIDAR_MODEL={shown(raw)} {REFUSED_MODELS[key]}")
        elif key not in MODELS:
            problems.append(f"LIDAR_MODEL={shown(raw)} is not a Seyond model this image runs; "
                            "accepted (any case, optional 'seyond-' prefix, '-' and '_' ignored): "
                            f"{model_list()}")
        else:
            model = MODELS[key]
    get("LIDAR_MODEL")

    # SENSOR_IP
    sensor = None
    if "SENSOR_IP" not in env:
        problems.append("SENSOR_IP is not set; accepted: the sensor's own IPv4 address, for example "
                        "172.168.1.10 (the driver connects to it)")
    elif "SENSOR_IP" in present:
        sensor = _ipv4("SENSOR_IP", present["SENSOR_IP"], problems, "the sensor's own IPv4 address")
    get("SENSOR_IP")

    # SENSOR_ID
    sensor_id = None
    value = get("SENSOR_ID")
    if value:
        if not _SENSOR_ID_RE.fullmatch(value) or "__" in value:
            problems.append(f"SENSOR_ID={shown(value)} is not a sensor id; accepted: 1-31 characters, "
                            "a lowercase letter then lowercase letters, digits or single '_' (it "
                            "becomes the node's ROS namespace)")
        else:
            sensor_id = value

    # DATA_PORT / DEVICE_PORT. The vendor default uses 8010 for both: the UDP port on this host and
    # the sensor's TCP command port are different sockets, so equal values are fine.
    data_port = DEFAULT_DATA_PORT
    value = get("DATA_PORT")
    if value:
        port = _port("DATA_PORT", value, problems, f"the UDP port the sensor sends to, default "
                     f"{DEFAULT_DATA_PORT}")
        data_port = port if port is not None else data_port
    device_port = DEFAULT_DEVICE_PORT
    value = get("DEVICE_PORT")
    if value:
        port = _port("DEVICE_PORT", value, problems, f"the sensor's TCP command port, default "
                     f"{DEFAULT_DEVICE_PORT}")
        device_port = port if port is not None else device_port

    # HOST_IP
    host_ip = None
    value = get("HOST_IP")
    if value:
        address, _, prefix = value.partition("/")
        if "/" in value and not _PREFIX_RE.fullmatch(prefix):
            problems.append(f"HOST_IP={shown(value)} has no usable prefix length; accepted: the "
                            "host's IPv4 address on the sensor network, optionally with /1-/32, for "
                            "example 172.168.1.100/24")
        else:
            host = _ipv4("HOST_IP", address, problems,
                         "the host's IPv4 address on the sensor network (optionally with /1-/32)")
            if host is not None:
                host_ip = value
                if sensor is not None and host == sensor:
                    problems.append(f"HOST_IP={shown(value)} is the sensor's own address; accepted: "
                                    "the host's address on the sensor network")
                    host_ip = None
                elif sensor is not None and prefix:
                    network = ipaddress.IPv4Interface(value).network
                    if sensor not in network:
                        problems.append(f"HOST_IP={shown(value)}: the sensor at {sensor} is not in "
                                        f"the host's subnet {network}; accepted: a host address in "
                                        "the sensor's subnet")
                        host_ip = None

    # FRAME_ID
    frame_id = DEFAULT_FRAME_ID
    value = get("FRAME_ID")
    if value:
        if not _FRAME_RE.fullmatch(value):
            problems.append(f"FRAME_ID={shown(value)} is not a frame id; accepted: 1-64 characters, "
                            "letters, digits and '_', not starting with a digit (default "
                            f"{DEFAULT_FRAME_ID})")
        else:
            frame_id = value

    # POINTCLOUD_TOPIC
    topic = DEFAULT_TOPIC
    value = get("POINTCLOUD_TOPIC")
    if value:
        if not _TOPIC_RE.fullmatch(value) or "__" in value or len(value) > 200:
            problems.append(f"POINTCLOUD_TOPIC={shown(value)} is not a ROS topic name; accepted: "
                            "'/'-separated tokens of letters, digits and single '_', each starting "
                            "with a letter, absolute (/x) or relative to the SENSOR_ID namespace "
                            f"(default {DEFAULT_TOPIC})")
        else:
            topic = value

    # TIME_MODE
    time_mode = DEFAULT_TIME_MODE
    value = get("TIME_MODE")
    if value:
        if value.lower() == "host":
            problems.append(f"TIME_MODE={shown(value)} is not supported by this image; accepted: "
                            "sensor (the default), gps, ptp or ntp. The Seyond driver always stamps "
                            "clouds with the sensor's clock; synchronise the sensor instead")
        elif value.lower() not in TIME_MODES:
            problems.append(f"TIME_MODE={shown(value)} is not a time mode; accepted, any case: "
                            "sensor (the default), gps, ptp or ntp (all stamp with the sensor's own "
                            "time; the sensor must be synchronised that way, this image does not "
                            "program it)")
        else:
            time_mode = value.lower()

    # PUBLISH_SCAN
    value = get("PUBLISH_SCAN")
    if value:
        flag = _bool("PUBLISH_SCAN", value, problems, "only a false value is accepted here")
        if flag:
            problems.append(f"PUBLISH_SCAN={shown(value)} is not supported by this image; accepted: "
                            "unset or false. The Seyond driver publishes PointCloud2 only")

    # SEYOND_RUN_DIR
    run_dir = DEFAULT_RUN_DIR
    value = get("SEYOND_RUN_DIR")
    if value:
        if not _RUN_DIR_RE.fullmatch(value) or "/../" in value + "/":
            problems.append(f"SEYOND_RUN_DIR={shown(value)} is not an absolute directory path; "
                            "accepted: an absolute path of letters, digits, '_', '.', '-' and '/' "
                            f"(default {DEFAULT_RUN_DIR})")
        else:
            run_dir = value

    # DUAL_LIDAR
    value = env.get("DUAL_LIDAR")
    if value:
        flag = _bool("DUAL_LIDAR", value, problems, "only a false value is accepted here")
        if flag:
            problems.append(f"DUAL_LIDAR={shown(value)} is not supported with {CONTRACT_SWITCH}; "
                            "accepted: unset or false. Run one container per Seyond sensor (each "
                            "with its own DATA_PORT)")

    # Unknown variables under this image's prefixes.
    for name in sorted(env):
        if name in KNOWN or not name.startswith(STRICT_PREFIXES):
            continue
        close = [ALIASES[name]] if name in ALIASES else difflib.get_close_matches(name, KNOWN, n=1,
                                                                                    cutoff=0.6)
        hint = f" (did you mean {close[0]}?)" if close else ""
        problems.append(f"{name} is not a variable this image knows{hint}; accepted: "
                        f"{', '.join(KNOWN)}")

    if problems:
        raise Refused(problems)
    assert model is not None and sensor is not None
    return Settings(model=model, sensor_ip=str(sensor), sensor_id=sensor_id, data_port=data_port,
                    device_port=device_port, host_ip=host_ip, frame_id=frame_id, topic=topic,
                    time_mode=time_mode, run_dir=run_dir, sources=sources)


# --------------------------------------------------------------------------------------------
# Rendering: the vendor's config.yaml keys in the vendor's order, vendor values except the ones the
# environment sets and three fixed choices (marked).
# --------------------------------------------------------------------------------------------

ENV_SET = object()

COMMON_PARAMS: list[tuple[str, object]] = [
    ("log_level", "info"), ("fusion_enable", False), ("fusion_topic", "/iv_points_fusion"),
    ("fusion_time_window", 50), ("fusion_buffer_size", 10),
]

LIDAR_PARAMS: list[tuple[str, object]] = [
    ("replay_rosbag", False), ("packet_mode", False), ("aggregate_num", 20),
    ("frame_id", ENV_SET), ("packet_topic", "iv_packets"),  # relative (vendor: /iv_packets)
    ("frame_topic", ENV_SET),
    ("lidar_name", ENV_SET), ("lidar_ip", ENV_SET), ("port", ENV_SET), ("udp_port", ENV_SET),
    ("reflectance_mode", True), ("multiple_return", 1), ("enable_falcon_ring", False),
    ("enable_imu_msg", False),
    ("continue_live", True),  # fixed: false means one connection attempt, then a deaf node
    ("inno_pc_file", ""), ("pcap_file", ""), ("hv_table_file", ""), ("packet_rate", 10000),
    ("file_rewind", 0),
    ("max_range", 2000.0), ("min_range", 0.1), ("coordinate_mode", 3),
    ("transform_enable", False), ("x", 0.0), ("y", 0.0), ("z", 0.0), ("pitch", 0.0), ("yaw", 0.0),
    ("roll", 0.0), ("transform_matrix", ""),
]


def lidar_parameters(s: Settings) -> list[tuple[str, object]]:
    from_env = {
        "frame_id": s.frame_id,
        "frame_topic": s.topic,
        "lidar_name": s.sensor_id or "seyond",
        "lidar_ip": s.sensor_ip,
        "port": s.device_port,
        "udp_port": s.data_port,  # >= 0 selects UDP data (driver_lidar.cc); never TCP data
    }
    return [(k, from_env[k] if v is ENV_SET else v) for k, v in LIDAR_PARAMS]


def yaml_scalar(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        text = repr(value)
        return text if ("." in text or "e" in text) else text + ".0"
    if isinstance(value, str):
        return json.dumps(value)  # a JSON string is a YAML double-quoted scalar
    raise TypeError(f"cannot render {value!r}")


def render(s: Settings) -> str:
    lines = [
        f"# Rendered from the environment by the {PROG} image ({CONTRACT_SWITCH}=1); do not edit.",
        f"# LIDAR_MODEL {s.model}: the SDK identifies the sensor from its packets.",
        f"# TIME_MODE {s.time_mode}: clouds carry the sensor's own time (the driver has no host-time "
        "option); the sensor must be synchronised that way, this image does not program it.",
    ]
    if s.host_ip:
        lines.append(f"# HOST_IP {s.host_ip}: checked against SENSOR_IP; the driver does not use it")
    lines.append("common:")
    for name, value in COMMON_PARAMS:
        lines.append(f"  {name}: {yaml_scalar(value)}")
    lines.append("lidars:")
    lines.append("  - lidar:")
    for name, value in lidar_parameters(s):
        lines.append(f"      {name}: {yaml_scalar(value)}")
    return "\n".join(lines) + "\n"


def node_argv(s: Settings) -> list[str]:
    argv = [DRIVER_NODE, "--ros-args"]
    if s.sensor_id:
        argv += ["-r", f"__ns:={s.namespace}"]
    argv += ["-p", f"config_path:={s.config_path}"]
    return argv


def describe(s: Settings) -> dict:
    return {
        "model": s.model,
        "node": s.node_fqn,
        "config_path": s.config_path,
        "common": dict(COMMON_PARAMS),
        "lidar": dict(lidar_parameters(s)),
        "time_mode": s.time_mode,
        "host_ip": s.host_ip,
        "argv": node_argv(s),
        "sources": dict(sorted(s.sources.items())),
    }


# --------------------------------------------------------------------------------------------
# Command line.
# --------------------------------------------------------------------------------------------


def refuse(problems: list[str], as_json: bool = False) -> int:
    if as_json:
        print(json.dumps({"refused": problems}, indent=2))
    for problem in problems:
        print(f"{PROG}: refusing to start: {problem}", file=sys.stderr)
    return EXIT_REFUSED


def write_config(s: Settings, text: str) -> None:
    os.makedirs(s.run_dir, mode=0o700, exist_ok=True)
    tmp = s.config_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        handle.write(text)
    os.replace(tmp, s.config_path)


def exec_env(env: Mapping[str, str]) -> dict[str, str]:
    out = dict(env)
    out.setdefault("ROS_HOME", DEFAULT_ROS_HOME)
    out.setdefault("ROS_LOG_DIR", out["ROS_HOME"].rstrip("/") + "/log")
    return out


def cmd_run(env: Mapping[str, str]) -> int:
    try:
        s = parse(env)
    except Refused as exc:
        return refuse(exc.problems)
    text = render(s)
    child_env = exec_env(env)
    try:
        write_config(s, text)
        os.makedirs(child_env["ROS_LOG_DIR"], exist_ok=True)
    except OSError as exc:
        return refuse([f"cannot write {exc.filename or s.run_dir} ({exc.strerror}) as uid "
                       f"{os.getuid()}; accepted: a writable /tmp, for example docker run "
                       "--tmpfs /tmp or compose 'tmpfs: [/tmp]' with a read-only root filesystem "
                       "(or point SEYOND_RUN_DIR and ROS_LOG_DIR at a writable volume)"])
    print(f"{PROG}: Seyond {s.model} at {s.sensor_ip}, data port {s.data_port}, command port "
          f"{s.device_port}, node {s.node_fqn}, topic {s.topic}, configuration {s.config_path}",
          file=sys.stderr, flush=True)
    argv = node_argv(s)
    try:
        os.execve(argv[0], argv, child_env)
    except OSError as exc:
        print(f"{PROG}: cannot start the driver {argv[0]}: {exc.strerror} (a broken image)",
              file=sys.stderr)
    return EXIT_REFUSED


def cmd_check(env: Mapping[str, str], as_json: bool) -> int:
    try:
        s = parse(env)
    except Refused as exc:
        return refuse(exc.problems, as_json)
    if as_json:
        out = describe(s)
        out["config_yaml"] = render(s)
        print(json.dumps(out, indent=2))
    else:
        sys.stdout.write(render(s))
        print("# exec: " + " ".join(node_argv(s)))
    return EXIT_OK


def cmd_selfcheck(node: str | None = None) -> int:
    problems = []
    try:
        render(parse({CONTRACT_SWITCH: "1", "LIDAR_MODEL": "robin-w", "SENSOR_IP": "172.168.1.10"}))
    except Refused as exc:
        problems += exc.problems
    node = node or DRIVER_NODE
    if not os.access(node, os.X_OK):
        problems.append(f"driver binary missing or not executable: {node}")
    elif os.stat(node).st_mode & 0o005 != 0o005:
        # The contract runs as any uid (97000 in the product).
        problems.append(f"driver binary not readable and executable by every uid: {node}")
    for p in problems:
        print(f"{PROG}: selfcheck FAILED: {p}", file=sys.stderr)
    if not problems:
        print(f"{PROG}: selfcheck: renders, {node} present")
    return EXIT_REFUSED if problems else EXIT_OK


# The image's default command (legacy). With the contract switched on it means "run".
LEGACY_DEFAULT_CMD = ["ros2", "launch", "seyond", "start_with_config.py"]
USAGE = f"usage: {PROG} [run | check [--json] | selfcheck]  (with {CONTRACT_SWITCH}=1; see AGENTS.md)"


def main(argv: list[str], env: Mapping[str, str]) -> int:
    if not argv or argv == ["run"] or argv == LEGACY_DEFAULT_CMD:
        return cmd_run(env)
    if argv[0] == "check" and argv[1:] in ([], ["--json"]):
        return cmd_check(env, argv[1:] == ["--json"])
    if argv == ["selfcheck"]:
        return cmd_selfcheck()
    if argv[0] in ("-h", "--help", "help"):
        print(USAGE)
        return EXIT_OK
    print(f"{PROG}: refusing to start: unknown command {shown(' '.join(argv))} with "
          f"{CONTRACT_SWITCH}=1; accepted: run (the default), check, check --json. The contract "
          "renders the configuration from the environment; to run something else, unset "
          f"{CONTRACT_SWITCH} or override the entrypoint", file=sys.stderr)
    return EXIT_USAGE


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:], os.environ))

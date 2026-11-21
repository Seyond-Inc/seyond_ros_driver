"""Unit tier: the contract against the driver source it relies on, read as text (no ROS, no build).

If a vendor update or a Dockerfile change moves any of these, the contract would silently stop
working; these tests turn that into a failure.
"""

from __future__ import annotations

import re
from pathlib import Path

import yaml

import seyond_env as s

REPO = Path(__file__).resolve().parents[2]
PKG = REPO / "src" / "seyond_lidar_ros"
YAML_TOOLS = (PKG / "src" / "driver" / "yaml_tools.hpp").read_text()
ADAPTER = (PKG / "src" / "driver" / "ros2_driver_adapter.hpp").read_text()
DRIVER = (PKG / "src" / "driver" / "driver_lidar.cc").read_text()
CMAKE = (PKG / "CMakeLists.txt").read_text()
VENDOR_CONFIG = yaml.safe_load((PKG / "config" / "config.yaml").read_text())
DOCKERFILE = (REPO / "Dockerfile").read_text()
BUILD = (REPO / "build.bash").read_text()
PY_TYPES = {"bool": bool, "int32_t": int, "uint32_t": int, "double": float, "std::string": str}
BASE = {"METROLLA_DRIVER_CONTRACT": "1", "LIDAR_MODEL": "robin-w", "SENSOR_IP": "172.168.1.10"}


def reads(section: str) -> dict[str, type]:
    pattern = (r'\["' + section + r'"\]\["(\w+)"\]\.as<([\w:]+)>') if section == "lidar" else \
        (r'config_\["common"\]\["(\w+)"\]\.as<([\w:]+)>')
    return {k: PY_TYPES[t] for k, t in re.findall(pattern, YAML_TOOLS)}


def test_every_rendered_key_is_read_by_the_driver_with_that_type():
    doc = yaml.safe_load(s.render(s.parse(BASE)))
    for section, values in (("common", doc["common"]), ("lidar", doc["lidars"][0]["lidar"])):
        known = reads(section)
        for key, value in values.items():
            assert key in known, f"{section}.{key} is not read by yaml_tools.hpp"
            assert isinstance(value, known[key]) and (known[key] is bool or not isinstance(value, bool)), \
                (section, key, value, known[key])


def test_every_key_the_driver_reads_is_rendered():
    doc = yaml.safe_load(s.render(s.parse(BASE)))
    assert set(reads("common")) == set(doc["common"])
    assert set(reads("lidar")) == set(doc["lidars"][0]["lidar"])


def test_defaults_are_the_vendor_config_except_the_deliberate_ones():
    doc = yaml.safe_load(s.render(s.parse(BASE)))
    vendor = VENDOR_CONFIG["lidars"][0]["lidar"]
    ours = doc["lidars"][0]["lidar"]
    differ = {k for k in ours if vendor.get(k) != ours[k]}
    # Relative topics (they follow SENSOR_ID; without it they are the vendor's absolute names).
    assert differ == {"packet_topic", "frame_topic"}
    assert vendor["packet_topic"] == "/" + ours["packet_topic"]
    assert vendor["frame_topic"] == "/" + ours["frame_topic"]
    for key, value in VENDOR_CONFIG["common"].items():
        assert doc["common"][key] == value


def test_node_reads_config_path_and_is_named_seyond():
    assert 'make_shared("seyond"' in ADAPTER
    assert re.search(r'get_parameter_or<std::string>\("config_path", yaml_file, ""\)', ADAPTER)
    assert "YamlTools::parseConfig(yaml_file" in ADAPTER
    assert s.NODE_NAME == "seyond"


def test_udp_port_selects_udp_data_and_continue_live_gates_the_start_retry():
    assert re.search(r"if \(param_\.udp_port >= 0\) \{\s*protocol_ = INNO_LIDAR_PROTOCOL_PCS_UDP;", DRIVER)
    assert re.search(r"while \(start_val != 0 && param_\.continue_live && is_running_\);", DRIVER)


def test_clouds_carry_the_sensors_time():
    """Why TIME_MODE=host is refused: the frame stamp is the packets' ts_start_us."""
    assert "frame_data.timestamp = frame_start_ts_;" in DRIVER
    assert "frame_start_ts_ = pkt->common.ts_start_us;" in DRIVER
    assert "int64_t ts_ns = timestamp * 1000;" in ADAPTER


def test_image_layout_matches_the_contracts_paths():
    assert "project(seyond)" in CMAKE
    assert re.search(r"install\(TARGETS\s+seyond_node\s+seyond_test\s+DESTINATION lib/\$\{PROJECT_NAME\}", CMAKE)
    assert "WORKDIR /opt/metrolla/seyond" in DOCKERFILE and "COPY . ." in DOCKERFILE
    assert "colcon build\n" in BUILD and "--merge-install" not in BUILD  # install/seyond/...
    assert s.DRIVER_NODE == "/opt/metrolla/seyond/install/seyond/lib/seyond/seyond_node"
    assert "python3 -I docker/seyond_env.py selfcheck" in DOCKERFILE
    assert 'ENTRYPOINT ["entrypoint.sh"]' in DOCKERFILE
    assert 'CMD ["ros2", "launch", "seyond", "start_with_config.py"]' in DOCKERFILE
    assert s.LEGACY_DEFAULT_CMD == ["ros2", "launch", "seyond", "start_with_config.py"]


def test_a_config_the_driver_cannot_parse_exits_0():
    """Pinned because it matters for restart policies: on a YAML parse error seyond_node exits
    with status 0 (restart: on-failure would not restart it). The contract renders YAML that
    always parses (every unit and regression test loads it)."""
    assert re.search(r'ROS_ERROR\("Parse config file failed"\);\s*exit\(0\);', ADAPTER)

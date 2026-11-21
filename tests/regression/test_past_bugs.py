"""Past bugs of this image, pinned so they cannot come back in contract mode (TESTS.md lists the
legacy ones, tested by tests/test_*.py).

1. continue_live: false made a deaf node (one connection attempt); TCP data has no mid-run
   reconnect since the 2026-08-31 vendor sync. The contract always renders continue_live: true and
   UDP data (udp_port = DATA_PORT >= 1024), for any environment it accepts.
2. DUAL_LIDAR was read as always true (v1.0.5-dual). The contract accepts only false values.
3. The rendered configuration must be the file the node reads: seyond_node takes config_path from
   its ROS parameter, and the contract passes the file it wrote.
"""

from __future__ import annotations

import itertools
from pathlib import Path

import pytest
import yaml

import seyond_env as s
from harness import run

REPO = Path(__file__).resolve().parents[2]
BASE = {"METROLLA_DRIVER_CONTRACT": "1", "LIDAR_MODEL": "robin-w", "SENSOR_IP": "172.168.1.10"}


def test_bug1_continue_live_and_udp_for_every_accepted_environment():
    options = {
        "SENSOR_ID": [None, "east"], "DATA_PORT": [None, "1024", "8011", "65535"],
        "DEVICE_PORT": [None, "8010"], "TIME_MODE": [None, "ptp"], "DUAL_LIDAR": [None, "", "false"],
    }
    count = 0
    for values in itertools.product(*options.values()):
        env = dict(BASE)
        env.update({k: v for k, v in zip(options, values) if v is not None})
        lidar = yaml.safe_load(s.render(s.parse(env)))["lidars"][0]["lidar"]
        assert lidar["continue_live"] is True
        assert lidar["udp_port"] >= 1024 and lidar["inno_pc_file"] == "" and lidar["pcap_file"] == ""
        count += 1
    assert count == 2 * 4 * 2 * 2 * 3


@pytest.mark.parametrize("value", ["1", "true", "True", "yes", "on"])
def test_bug2_dual_lidar_true_is_refused(value):
    with pytest.raises(s.Refused):
        s.parse(dict(BASE, DUAL_LIDAR=value))


def test_bug3_the_node_reads_the_rendered_file(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(s.os, "execve", lambda path, argv, envp: calls.append(argv))
    env = dict(BASE, FRAME_ID="seyond_east", SEYOND_RUN_DIR=str(tmp_path), ROS_HOME=str(tmp_path))
    s.main(["run"], env)
    (argv,), = [calls]
    assert argv[argv.index("-p") + 1] == f"config_path:={tmp_path}/config.yaml"
    assert 'frame_id: "seyond_east"' in (tmp_path / "config.yaml").read_text()


def test_switch_dispatch_through_the_entrypoint():
    record = run(REPO / "entrypoint.sh", dict(BASE), ["ros2", "launch", "seyond", "start_with_config.py"])
    assert record["exit"] == 0 and record["stdout"] == "" and record["stderr"] == ""
    assert record["exec"]["argv"] == ["contract-entrypoint.sh", "ros2", "launch", "seyond",
                                      "start_with_config.py"]


def test_images_without_the_contract_refuse_the_switch(tmp_path):
    """The -hb image copies entrypoint.sh but not docker/; the -unified image has the same block
    in entrypoint-unified.sh. Simulated by removing the contract script from the sandbox."""
    script = tmp_path / "entrypoint.sh"
    script.write_text((REPO / "entrypoint.sh").read_text().replace(
        "/opt/metrolla/seyond/docker/contract-entrypoint.sh", str(tmp_path / "absent.sh")))
    record = run(script, dict(BASE), [])
    assert record["exit"] == 1 and record["exec"] is None
    assert record["stderr"].startswith("seyond-driver: refusing to start: METROLLA_DRIVER_CONTRACT=1 is set")

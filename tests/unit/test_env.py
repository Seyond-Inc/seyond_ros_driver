"""Unit tier: the environment contract of docker/seyond_env.py (METROLLA_DRIVER_CONTRACT=1)."""

from __future__ import annotations

import json

import pytest
import yaml

import seyond_env as s

BASE = {"METROLLA_DRIVER_CONTRACT": "1", "LIDAR_MODEL": "robin-w", "SENSOR_IP": "172.168.1.10"}


def env(**kw):
    out = dict(BASE)
    for k, v in kw.items():
        if v is None:
            out.pop(k, None)
        else:
            out[k] = v
    return out


def refused(e) -> list[str]:
    with pytest.raises(s.Refused) as exc:
        s.parse(e)
    assert exc.value.problems, "refused without a reason"
    return exc.value.problems


def rendered(e) -> dict:
    return yaml.safe_load(s.render(s.parse(e)))


def lidar(e) -> dict:
    doc = rendered(e)
    assert len(doc["lidars"]) == 1
    return doc["lidars"][0]["lidar"]


# ---------------------------------------------------------------- accepted values and defaults


def test_minimal_environment_defaults():
    st = s.parse(BASE)
    assert (st.model, st.sensor_ip, st.sensor_id) == ("Robin-W", "172.168.1.10", None)
    assert (st.data_port, st.device_port) == (8010, 8010)
    assert (st.frame_id, st.topic, st.time_mode) == ("seyond", "iv_points", "sensor")
    assert st.config_path == "/tmp/seyond/config.yaml" and st.node_fqn == "/seyond"


def test_minimal_render_is_the_vendor_config_with_the_contract_keys():
    li = lidar(BASE)
    assert li["lidar_ip"] == "172.168.1.10" and li["port"] == 8010 and li["udp_port"] == 8010
    assert li["frame_id"] == "seyond" and li["frame_topic"] == "iv_points" and li["lidar_name"] == "seyond"
    assert li["continue_live"] is True
    assert li["inno_pc_file"] == "" and li["pcap_file"] == ""  # live sensor, never a file


@pytest.mark.parametrize("raw,model", [
    ("robin-w", "Robin-W"), ("seyond-robin-w", "Robin-W"), ("Robin-W", "Robin-W"), ("ROBIN_W", "Robin-W"),
    ("robinw", "Robin-W"), ("robin-elite", "Robin-ELITE"), ("robin-e1x", "Robin-ELITE"),
    ("robin-e2", "Robin-E2"), ("falcon-k1", "Falcon-K1"), ("falcon-k2", "Falcon-K2"),
    ("falcon-k24", "Falcon-K24"),
])
def test_model_names(raw, model):
    assert s.parse(env(LIDAR_MODEL=raw)).model == model


def test_full_environment():
    li = lidar(env(LIDAR_MODEL="seyond-robin-w", SENSOR_IP="10.9.0.10", SENSOR_ID="dock_west",
                   DATA_PORT="8011", DEVICE_PORT="8012", HOST_IP="10.9.0.2/24", FRAME_ID="seyond_west",
                   POINTCLOUD_TOPIC="/iv_points2", TIME_MODE="PTP", PUBLISH_SCAN="0", DUAL_LIDAR="false"))
    assert (li["lidar_ip"], li["udp_port"], li["port"]) == ("10.9.0.10", 8011, 8012)
    assert (li["frame_id"], li["frame_topic"], li["lidar_name"]) == ("seyond_west", "/iv_points2", "dock_west")


def test_udp_data_always_and_continue_live_always():
    for e in (BASE, env(DATA_PORT="1024"), env(DATA_PORT="65535", SENSOR_ID="a")):
        li = lidar(e)
        assert li["udp_port"] >= 1024 and li["continue_live"] is True


def test_packet_topic_is_relative():
    assert lidar(env(SENSOR_ID="west"))["packet_topic"] == "iv_packets"


@pytest.mark.parametrize("mode", ["sensor", "gps", "ptp", "NTP"])
def test_time_modes(mode):
    assert s.parse(env(TIME_MODE=mode)).time_mode == mode.lower()


def test_metrolla_and_ros_variables_pass_through():
    s.parse(env(METROLLA_SITE="x", ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                FASTRTPS_DEFAULT_PROFILES_FILE="/etc/metrolla/fastdds.xml", TZ="UTC", HOME="/tmp"))


# ---------------------------------------------------------------- refusals


def test_nothing_set():
    problems = refused({"METROLLA_DRIVER_CONTRACT": "1"})
    assert [p.split(" ")[0] for p in problems] == ["LIDAR_MODEL", "SENSOR_IP"]


@pytest.mark.parametrize("raw,needle", [
    ("robin-x", "is not a Seyond model"), ("hb-d1", "Hummingbird"), ("hummingbird", "Hummingbird"),
    ("robin", "ambiguous"), ("falcon", "ambiguous"), ("robin-w ", "is not a Seyond model"),
    ("", "set but empty"), ("hesai-xt32", "is not a Seyond model"),
])
def test_bad_models(raw, needle):
    assert any(p.startswith("LIDAR_MODEL") and needle in p for p in refused(env(LIDAR_MODEL=raw)))


def test_host_time_is_refused_with_the_reason():
    problems = refused(env(TIME_MODE="host"))
    assert "always stamps clouds with the sensor's clock" in problems[0]


@pytest.mark.parametrize("value", ["172.168.1", "172.168.1.256", "172.168.01.10", "0.0.0.0",
                                   "239.1.1.1", "255.255.255.255", "lidar", "172.168.1.10/24"])
def test_bad_sensor_ip(value):
    assert any(p.startswith("SENSOR_IP=") for p in refused(env(SENSOR_IP=value)))


@pytest.mark.parametrize("name", ["DATA_PORT", "DEVICE_PORT"])
@pytest.mark.parametrize("value", ["0", "80", "1023", "65536", "08010", "x", "-1"])
def test_bad_ports(name, value):
    assert any(p.startswith(f"{name}=") and "1024-65535" in p for p in refused(env(**{name: value})))


def test_host_ip_checks():
    assert any("not in the host's subnet" in p for p in refused(env(HOST_IP="10.0.0.1/24")))
    assert any("sensor's own address" in p for p in refused(env(HOST_IP="172.168.1.10/24")))
    s.parse(env(HOST_IP="172.168.1.100/24"))


@pytest.mark.parametrize("name,value", [
    ("SENSOR_ID", "West"), ("SENSOR_ID", "w__1"), ("FRAME_ID", "1x"), ("POINTCLOUD_TOPIC", "iv points"),
    ("TIME_MODE", "tai"), ("PUBLISH_SCAN", "true"), ("PUBLISH_SCAN", "x"), ("DUAL_LIDAR", "True"),
    ("DUAL_LIDAR", "1"), ("SEYOND_RUN_DIR", "run"), ("SEYOND_RUN_DIR", "/a/../b"),
])
def test_bad_values(name, value):
    assert any(p.startswith(name) for p in refused(env(**{name: value})))


@pytest.mark.parametrize("name,hint", [
    ("LIDAR_IPS", "SENSOR_IP"), ("LIDAR_IP", "SENSOR_IP"), ("UDP_PORT", "DATA_PORT"),
    ("LIDAR_PORT", "DEVICE_PORT"), ("CONTINUE_LIVE", None), ("SEYOND_RUNDIR", "SEYOND_RUN_DIR"),
    ("POINT_CLOUD_TOPIC", "POINTCLOUD_TOPIC"), ("SENSOR_PORT", "DATA_PORT"),
])
def test_typos_and_other_drivers_names(name, hint):
    problems = refused(env(**{name: "x"}))
    match = [p for p in problems if p.startswith(f"{name} is not a variable this image knows")]
    assert match, problems
    if hint:
        assert f"did you mean {hint}?" in match[0]


@pytest.mark.parametrize("value", ["2", "true", "yes", "on"])
def test_unknown_contract_version(value):
    assert refused(env(METROLLA_DRIVER_CONTRACT=value))[0].startswith("METROLLA_DRIVER_CONTRACT=")


def test_all_problems_at_once():
    assert len(refused({"METROLLA_DRIVER_CONTRACT": "1", "LIDAR_MODEL": "x", "SENSOR_IP": "x",
                        "TIME_MODE": "host", "DATA_PORT": "1"})) == 4


# ---------------------------------------------------------------- argv, check, run


def test_argv():
    assert s.node_argv(s.parse(BASE)) == [s.DRIVER_NODE, "--ros-args", "-p",
                                          "config_path:=/tmp/seyond/config.yaml"]
    st = s.parse(env(SENSOR_ID="west"))
    assert s.node_argv(st)[2:4] == ["-r", "__ns:=/west"] and st.node_fqn == "/west/seyond"


def test_check_and_json(capsys):
    assert s.main(["check"], BASE) == 0
    out = capsys.readouterr().out
    assert out.splitlines()[-1] == "# exec: " + " ".join(s.node_argv(s.parse(BASE)))
    assert yaml.safe_load(out)["lidars"][0]["lidar"]["lidar_ip"] == "172.168.1.10"
    assert s.main(["check", "--json"], env(SENSOR_ID="west")) == 0
    j = json.loads(capsys.readouterr().out)
    assert j["node"] == "/west/seyond" and j["lidar"]["continue_live"] is True
    assert yaml.safe_load(j["config_yaml"]) == rendered(env(SENSOR_ID="west"))


def test_refusal_stderr(capsys):
    assert s.main(["check"], env(LIDAR_MODEL="hb-d1")) == 1
    err = capsys.readouterr().err.splitlines()
    assert len(err) == 1 and err[0].startswith("seyond-driver: refusing to start: LIDAR_MODEL='hb-d1'")


def test_unknown_command(capsys):
    assert s.main(["bash"], BASE) == 1
    assert s.main(["ros2", "launch", "seyond", "start_with_config.py",
                   "config_path:=/etc/metrolla-driver/config.yaml"], BASE) == 1
    assert "unknown command" in capsys.readouterr().err


def test_run_writes_the_file_the_node_is_given(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(s.os, "execve", lambda path, argv, envp: calls.append((path, argv, envp)))
    e = env(SEYOND_RUN_DIR=str(tmp_path / "run"), ROS_HOME=str(tmp_path / "ros"))
    assert s.main([], e) == 1
    (path, argv, envp), = calls
    assert path == s.DRIVER_NODE and argv[-1] == f"config_path:={tmp_path}/run/config.yaml"
    assert (tmp_path / "run" / "config.yaml").read_text() == s.render(s.parse(e))
    assert envp["ROS_LOG_DIR"] == f"{tmp_path}/ros/log"


def test_default_command_means_run(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(s.os, "execve", lambda *a: calls.append(a))
    s.main(s.LEGACY_DEFAULT_CMD, env(SEYOND_RUN_DIR=str(tmp_path), ROS_HOME=str(tmp_path)))
    assert len(calls) == 1


def test_run_unwritable(monkeypatch, capsys):
    monkeypatch.setattr(s.os, "execve", lambda *a: pytest.fail("must not exec"))
    assert s.main(["run"], env(SEYOND_RUN_DIR="/proc/seyond")) == 1
    assert "cannot write" in capsys.readouterr().err


def test_selfcheck(tmp_path):
    node = tmp_path / "seyond_node"
    node.write_text("#!/bin/sh\n")
    node.chmod(0o755)
    assert s.cmd_selfcheck(str(node)) == 0
    node.chmod(0o750)
    assert s.cmd_selfcheck(str(node)) == 1
    assert s.cmd_selfcheck(str(tmp_path / "missing")) == 1

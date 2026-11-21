"""Bug 4: continue_live must be true or the driver becomes a zombie.

With ``continue_live: false`` the driver makes exactly one connection attempt and never retries:
the start-retry loop in driver_lidar.cc is gated on it,
  ``while (start_val != 0 && param_.continue_live && is_running_)``
so a lidar that is unreachable at boot leaves a running but permanently deaf node. v1.0.7-dual
ships continue_live: true.

Mid-run recovery changed with the vendor sync of 2026-08-31 (upstream 18c5c93 "sync up with
internal version", merged in 592eca1): driver_lidar.cc no longer has the fatal-error reconnect
``if (fatal_error_ && param_.continue_live) { stop_lidar(); start_lidar(); }``. The old test that
pinned it failed on main from that merge on; the test was wrong, not the code. What recovers a
live driver now, for UDP data (udp_port >= 0: every shipped config and the opt-in contract), is the
SDK's UDP read loop: it never returns on silence and logs "recover from network timeout" when
packets come back. TCP data (udp_port < 0) has no mid-run reconnect any more, so it must not ship.
"""

import re

import pytest
import yaml

from conftest import CONFIG_YAML, DRIVER_LIDAR_CC, TWO_CONFIG_YAML, FIX_REASON  # noqa: F401


def _continue_live_values(path):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return [entry["lidar"]["continue_live"] for entry in data["lidars"]]


_CONFIG_VALUES = _continue_live_values(CONFIG_YAML)


@pytest.mark.xfail(
    not all(_CONFIG_VALUES),
    reason=(
        "config.yaml ships continue_live: false on origin/main => one "
        "connection attempt then zombie; flipped to true in v1.0.7-dual"
    ),
    strict=True,
)
def test_config_yaml_continue_live_true():
    """Auto-recovery requires continue_live: true for every lidar."""
    assert all(v is True for v in _CONFIG_VALUES), (
        f"continue_live values in config.yaml: {_CONFIG_VALUES}; false means "
        "no start retry and no fatal-error reconnect (zombie node)"
    )


def test_two_config_continue_live_true():
    """Dual config must also enable auto-recovery (both lidars)."""
    if not TWO_CONFIG_YAML.is_file():
        pytest.skip("two-config.yaml absent on this branch (pre-v1.0.7-dual)")
    values = _continue_live_values(TWO_CONFIG_YAML)
    assert len(values) >= 2 and all(v is True for v in values), (
        f"continue_live values in two-config.yaml: {values}"
    )


# --- source-level documentation of the zombie mechanism --------------------

def test_start_retry_loop_gated_on_continue_live(driver_src):
    """The connect-retry do/while loop only loops when continue_live is set."""
    assert re.search(
        r"while\s*\(\s*start_val\s*!=\s*0\s*&&\s*param_\.continue_live", driver_src
    ), (
        "expected the start-retry loop gate "
        "`while (start_val != 0 && param_.continue_live ...)` in driver_lidar.cc"
    )


SDK_READ = DRIVER_LIDAR_CC.parents[1] / "seyond_sdk" / "src" / "sdk_client" / "stage_client_read.cpp"


def _sdk_read_src():
    if not SDK_READ.is_file():
        pytest.fail("the seyond_sdk submodule is not checked out (git submodule update --init): "
                    "the SDK half of this check cannot run, which is an ABORT, not a pass")
    return SDK_READ.read_text(encoding="utf-8")


def test_fatal_error_reconnect_is_gone_and_udp_recovers_in_the_sdk(driver_src):
    """Past bug of this suite (failed on main since 2026-08-31): it pinned the vendor's removed
    fatal-error reconnect. If the vendor brings a reconnect back, this fails on purpose: re-read
    driver_lidar.cc and pin the new mechanism."""
    assert "fatal_error_" not in driver_src, (
        "driver_lidar.cc has a fatal_error_ path again: pin it (gated on continue_live?)"
    )
    # udp_port >= 0 selects UDP data (the SDK listens); < 0 TCP data.
    assert re.search(
        r"if\s*\(\s*param_\.udp_port\s*>=\s*0\s*\)\s*\{\s*protocol_\s*=\s*INNO_LIDAR_PROTOCOL_PCS_UDP;",
        driver_src,
    )
    sdk = _sdk_read_src()
    loop = sdk[sdk.index("int UdpInput::read_udp_("):sdk.index("int UdpInput::read_data()")]
    # A receive timeout (EAGAIN) continues the loop instead of ending the read stage ...
    assert re.search(r"if \(errno == EAGAIN \|\| errno == EWOULDBLOCK\) \{.*?continue;", loop, re.S)
    # ... and the next packet is taken as a recovery.
    assert 'inno_log_info("recover from network timeout");' in loop


@pytest.mark.parametrize("path", [CONFIG_YAML, TWO_CONFIG_YAML], ids=["config.yaml", "two-config.yaml"])
def test_shipped_configs_use_udp_data(path):
    """TCP data (udp_port < 0) has no mid-run reconnect since the vendor sync: never ship it."""
    if not path.is_file():
        pytest.skip(f"{path.name} absent on this branch")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    ports = [entry["lidar"]["udp_port"] for entry in data["lidars"]]
    assert ports and all(isinstance(p, int) and p >= 0 for p in ports), ports

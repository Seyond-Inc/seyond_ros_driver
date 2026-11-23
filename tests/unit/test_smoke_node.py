"""Graph readiness: delayed discovery, exact matching and bounded failure without DDS."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


@pytest.fixture
def checker(monkeypatch):
    elapsed = [0.0]
    spins = []

    def spin_once(node, timeout_sec):
        spins.append(timeout_sec)
        elapsed[0] += timeout_sec

    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace(spin_once=spin_once))
    path = Path(__file__).resolve().parents[2] / "docker/smoke/check_node.py"
    spec = importlib.util.spec_from_file_location("smoke_node", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: elapsed[0]))
    return module, elapsed, spins


def test_delayed_discovery(checker, capsys):
    module, elapsed, spins = checker
    node = SimpleNamespace(get_node_names_and_namespaces=lambda:
                           [("seyond", "/east")] if elapsed[0] >= 0.4 else [])
    assert module.wait_for_node(node, "/east/seyond", 1) == 0
    assert spins == [0.2, 0.2]
    assert capsys.readouterr().out.splitlines() == ["/east/seyond"]


@pytest.mark.parametrize("names", [[], [("seyond", "/")], [("seyond", "/west")],
                                  [("seyond_extra", "/east")]])
def test_missing_or_wrong_node_times_out(checker, capsys, names):
    module, elapsed, spins = checker
    node = SimpleNamespace(get_node_names_and_namespaces=lambda: names)
    assert module.wait_for_node(node, "/east/seyond", 0.5) == 1
    assert elapsed[0] == 0.5
    assert len(spins) == 3
    output = capsys.readouterr()
    assert not output.out
    assert "Timed out after 0.5 s waiting for node /east/seyond" in output.err
    expected = ", ".join(f"{ns.rstrip('/')}/{name}" for name, ns in names) or "(none)"
    assert f"last observed nodes: {expected}" in output.err


def test_immediate_discovery(checker):
    module, _, spins = checker
    node = SimpleNamespace(get_node_names_and_namespaces=lambda: [("seyond", "/east")])
    assert module.wait_for_node(node, "/east/seyond", 1) == 0
    assert spins == []


def test_graph_error_is_not_success(checker):
    module, _, _ = checker

    def broken_graph():
        raise RuntimeError("graph unavailable")

    node = SimpleNamespace(get_node_names_and_namespaces=broken_graph)
    with pytest.raises(RuntimeError, match="graph unavailable"):
        module.wait_for_node(node, "/east/seyond", 1)

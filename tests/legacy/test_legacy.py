"""Legacy mode stays exactly the released image's behaviour (v1.0.8-packets, what the fleets run).

Goldens: for each fleet environment and command in cases/*.env (from the balena-fleets compose
files and metrolla-edge's reference template), the record of running the entrypoint (exit code,
stdout, stderr, the exec'd command and its environment, and the YAML start_with_config.py would
load) as produced by the **released entrypoint** (entrypoint-v1.0.8-packets.sh, byte-identical
to the tag; the file has not changed since v1.0.0). The current entrypoint.sh must give the same.

REGEN=1 rewrites the goldens from the released script, never from the current one.
"""

from __future__ import annotations

import hashlib
import os
from pathlib import Path

import pytest

from harness import REPO, dump, read_case, run

HERE = Path(__file__).resolve().parent
CASES = sorted((HERE / "cases").glob("*.env"))
REFERENCE = HERE / "entrypoint-v1.0.8-packets.sh"
CURRENT = REPO / "entrypoint.sh"
RELEASED_BLOB = "50d7ff18c01dbdd4e84707eb433a571692981413"  # git rev-parse v1.0.8-packets:entrypoint.sh
SHEBANG = "#!/usr/bin/env bash\n"
BLOCK_END = "# --- end of the Metrolla contract block; the released entrypoint follows ---\n"


def git_blob_id(data: bytes) -> str:
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def test_reference_is_the_released_entrypoint():
    assert git_blob_id(REFERENCE.read_bytes()) == RELEASED_BLOB


def test_entrypoint_is_the_released_one_plus_the_opt_in_block_only():
    current = CURRENT.read_text()
    reference = REFERENCE.read_text()
    assert current.startswith(SHEBANG) and reference.startswith(SHEBANG)
    if current == reference:  # before the contract block was added
        return
    head, sep, tail = current.partition(BLOCK_END)
    assert sep, "entrypoint.sh differs from the release and has no contract block end marker"
    assert tail == reference[len(SHEBANG):], "the legacy part of entrypoint.sh changed"
    block = [line.strip() for line in head[len(SHEBANG):].splitlines() if not line.startswith("#")]
    assert block == [
        'if [ -n "${METROLLA_DRIVER_CONTRACT:-}" ] && [ "${METROLLA_DRIVER_CONTRACT}" != "0" ]; then',
        'if [ -x /opt/metrolla/seyond/docker/contract-entrypoint.sh ]; then',
        'exec /opt/metrolla/seyond/docker/contract-entrypoint.sh "$@"',
        'fi',
        'echo "seyond-driver: refusing to start: METROLLA_DRIVER_CONTRACT=${METROLLA_DRIVER_CONTRACT} '
        'is set but this image has no environment contract (only the main seyond_drivers image does); '
        'accepted: unset or 0 here" >&2',
        'exit 1',
        'fi',
    ]


def test_there_are_fleet_cases():
    assert len(CASES) >= 9


@pytest.mark.parametrize("case", CASES, ids=[c.stem for c in CASES])
def test_legacy_golden(case: Path):
    env, argv = read_case(case)
    golden = HERE / "golden" / (case.stem + ".json")
    reference = dump(run(REFERENCE, env, argv))
    if os.environ.get("REGEN"):
        golden.write_text(reference)
    expected = golden.read_text()
    assert reference == expected, "the released entrypoint no longer gives its golden (harness changed?)"
    assert dump(run(CURRENT, env, argv)) == expected, "legacy behaviour changed"

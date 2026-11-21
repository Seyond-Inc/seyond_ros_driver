"""Regression tier: goldens of `check` (rendered configuration and exec line) and of the refusal
messages, for the environments in cases/*.env. Names starting refused- must exit 1, others 0.

REGEN=1 rewrites the goldens; read the diff: a changed golden is a changed contract.
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

import seyond_env

HERE = Path(__file__).resolve().parent
CASES = sorted((HERE / "cases").glob("*.env"))


def read_env(path: Path) -> dict[str, str]:
    env = {}
    for line in path.read_text().splitlines():
        if line.strip() and not line.startswith("#"):
            key, sep, value = line.partition("=")
            assert sep, line
            env[key] = value
    return env


def test_there_are_cases():
    assert len([c for c in CASES if c.stem.startswith("refused-")]) >= 8
    assert len([c for c in CASES if not c.stem.startswith("refused-")]) >= 3


@pytest.mark.parametrize("case", CASES, ids=[c.stem for c in CASES])
def test_golden(case, capsys):
    code = seyond_env.main(["check"], read_env(case))
    out = capsys.readouterr()
    text = f"exit {code}\n--- stdout\n{out.out}--- stderr\n{out.err}"
    assert code == (1 if case.stem.startswith("refused-") else 0)
    golden = HERE / "golden" / (case.stem + ".golden")
    if os.environ.get("REGEN"):
        golden.write_text(text)
    assert text == golden.read_text()

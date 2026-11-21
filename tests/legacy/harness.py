"""Run an entrypoint script of this image outside the image, in a sandbox, and record what it does:
exit code, stdout, stderr, the command it execs (argv) and that command's environment, plus which
YAML the image's default launch file (start_with_config.py) would load for that environment.

The script's absolute paths (/opt/ros/jazzy, /opt/metrolla/seyond) are rewritten into a temporary
directory with empty setup.bash files, and its final `exec "$@"` is pointed at a recorder, so a
command such as `/bin/bash -lc "sleep 10; ..."` is recorded instead of run. The same rewrite is
applied to every script compared, and every sandbox path is mapped back in the record.
"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SHARE = "/opt/metrolla/seyond/install/seyond/share/seyond"
REWRITES = ("/opt/ros/jazzy", "/opt/metrolla/seyond")
BASE_ENV = {"PATH": "/usr/bin:/bin", "LANG": "C", "LC_ALL": "C"}
EXEC_LINE = 'exec "$@"'

RECORDER = """#!/usr/bin/env python3
import json, os, sys
with open(os.environ["HARNESS_RECORD"], "w") as f:
    json.dump({"argv": sys.argv[1:], "env": dict(os.environ)}, f)
"""


def launch_config(env: dict[str, str], argv: list[str]) -> str | None:
    """The YAML start_with_config.py loads (its DUAL_LIDAR rule, lifted from the launch file), for
    a command that launches it without a config_path:= argument; None otherwise."""
    command = " ".join(argv)
    if "start_with_config.py" not in command or "config_path:=" in command:
        return None
    sys.path.insert(0, str(REPO / "tests"))
    try:
        from conftest import extract_env_flag_callable  # the existing suite's AST helper
    finally:
        sys.path.pop(0)
    env_flag = extract_env_flag_callable()
    import os
    saved = os.environ.get("DUAL_LIDAR")
    try:
        if "DUAL_LIDAR" in env:
            os.environ["DUAL_LIDAR"] = env["DUAL_LIDAR"]
        else:
            os.environ.pop("DUAL_LIDAR", None)
        dual = env_flag("DUAL_LIDAR")
    finally:
        if saved is None:
            os.environ.pop("DUAL_LIDAR", None)
        else:
            os.environ["DUAL_LIDAR"] = saved
    return SHARE + ("/two-config.yaml" if dual else "/config.yaml")


def run(script: Path, env: dict[str, str], argv: list[str]) -> dict:
    with tempfile.TemporaryDirectory(prefix="seyond-legacy-") as tmp:
        root = Path(tmp)
        for setup in ("opt/ros/jazzy/setup.bash", "opt/metrolla/seyond/install/setup.bash"):
            (root / setup).parent.mkdir(parents=True, exist_ok=True)
            (root / setup).write_text("# empty in the sandbox\n")
        recorder = root / "recorder"
        recorder.write_text(RECORDER)
        recorder.chmod(0o755)
        # The contract block execs this when the switch is set: record that too.
        contract = root / "opt/metrolla/seyond/docker/contract-entrypoint.sh"
        contract.parent.mkdir(parents=True)
        contract.write_text(RECORDER.replace('sys.argv[1:]', '[os.path.basename(sys.argv[0])] + sys.argv[1:]'))
        contract.chmod(0o755)
        text = script.read_text()
        assert text.count(EXEC_LINE) == 1, "the entrypoint's final exec line changed"
        for prefix in REWRITES:
            text = text.replace(prefix, str(root) + prefix)
        text = text.replace(EXEC_LINE, f'exec "{recorder}" "$@"')
        sandboxed = root / "entrypoint.sh"
        sandboxed.write_text(text)
        record_file = root / "record.json"
        run_env = dict(BASE_ENV)
        run_env["HARNESS_RECORD"] = str(record_file)
        run_env.update(env)
        proc = subprocess.run(["bash", str(sandboxed), *argv], env=run_env, capture_output=True,
                              text=True, timeout=60, check=False)

        def back(value: str) -> str:
            return value.replace(str(root), "")

        record = None
        if record_file.exists():
            raw = json.loads(record_file.read_text())
            exec_env = {k: back(v) for k, v in raw["env"].items() if k in env or k == "PATH"}
            record = {"argv": [back(a) for a in raw["argv"]], "env": dict(sorted(exec_env.items()))}
        return {
            "exit": proc.returncode,
            "stdout": back(proc.stdout),
            "stderr": back(proc.stderr),
            "exec": record,
            "launch_config": launch_config(env, argv),
        }


def read_case(path: Path) -> tuple[dict[str, str], list[str]]:
    """KEY=VALUE lines (# comments); 'argv: ...' is the command as a JSON list (default: the
    image's CMD)."""
    env: dict[str, str] = {}
    argv = ["ros2", "launch", "seyond", "start_with_config.py"]
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("argv:"):
            argv = json.loads(line[len("argv:"):])
            continue
        key, sep, value = line.partition("=")
        assert sep, f"{path}: not KEY=VALUE: {line!r}"
        env[key] = value
    return env, argv


def dump(record: dict) -> str:
    return json.dumps(record, indent=2, sort_keys=True) + "\n"

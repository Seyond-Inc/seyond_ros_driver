# TESTS.md — seyond_ros_driver suite inventory

This repo wraps the Seyond (Innovusion) lidar ROS 2 driver into the edge
images run by the openBalena fleets. The regression-gate suite here is
**pure-Python contract tests**: it parses `launch/start_with_config.py`,
`CMakeLists.txt`, and the YAML configs — it never imports ROS and never
builds the driver, so it runs anywhere with `pytest + pyyaml` in under a
second.

The suite pins the four v1.0.5-dual field bugs (fixed 2026-08-15 in commits
`4a4e5f2` / `e7f10f0`, tagged **v1.0.7-dual**, currently on branch
`feat/dualdriver`). **Those fixes are NOT on `main` yet**, so on `main` the
fix-presence tests are **strict xfail** — they will automatically flip to
required-PASS the moment `feat/dualdriver` merges (a strict xfail that
starts passing errors the run, forcing the marker's removal).

1. **Always-true launch condition** — `if os.environ.get('DUAL_LIDAR') is
   True or 1 or "1":` parses as `(x is True) or 1 or "1"` and is constant
   true, so every dual image loaded `two-config.yaml` regardless of the env
   flag ⇒ crash-loop on every dual unit (seen at plant1
   precise-doorway).
2. **two-config.yaml never CMake-installed** — the launcher asked for
   `share/seyond/two-config.yaml` but the image never shipped it (the crash
   the always-true bug exposed).
3. **Lidar-2 topic typo** — `/iv_point2` vs downstream-expected
   `/iv_points2`: lidar-2 data silently vanishes when the second unit
   attaches.
4. **`continue_live: false`** — one connection attempt, then a zombie: both
   the start-retry loop and the fatal-error reconnect in `driver_lidar.cc`
   are gated on `continue_live`.

CI: `.github/workflows/regression-gate.yml` — `unit (launch/config contract tests)`
(Python 3.12, pytest + pyyaml, shellcheck, hadolint), `regression` (goldens and past bugs),
`smoke` (builds the image from source with the SDK submodule and runs `docker/smoke/smoke.sh`)
and `manifest-lint` (YAML configs parse with required lidar keys + `package.xml` parses).

## Suite inventory

| Suite | Layer | What it tests | Run command | Runtime | Requirements |
|---|---|---|---|---|---|
| `tests/test_dual_lidar_env.py` (9 tests) | unit, ROS-free | Bug 1: source-level pin that the always-true `is True or 1` pattern (and the `environ.get(...) is True` bug class) is absent; DUAL_LIDAR truth table (unset/`"0"`/`""`/`"false"` ⇒ single config, `"1"`/`"true"` ⇒ dual) against the `_env_flag` helper lifted out of the launch file via AST; missing-two-config fallback guard (no crash-loop) | `python3 -m pytest tests/test_dual_lidar_env.py -v` | < 1 s | Python 3.12, pytest |
| `tests/test_cmake_install.py` (3 tests) | unit, ROS-free | Bug 2: `config/two-config.yaml` present in an `install(FILES … DESTINATION share/…)` block; controls: `config.yaml` and `start_with_config.py` are installed | `python3 -m pytest tests/test_cmake_install.py -v` | < 1 s | Python 3.12, pytest |
| `tests/test_two_config_topics.py` (3 tests) | unit, ROS-free | Bug 3: `two-config.yaml` exists; lidar-2 `frame_topic` is exactly `/iv_points2` (and explicitly not the `/iv_point2` typo); control: lidar-1 stays `/iv_points` | `python3 -m pytest tests/test_two_config_topics.py -v` | < 1 s | Python 3.12, pytest, pyyaml |
| `tests/test_continue_live.py` (6 tests) | unit, ROS-free | Bug 4: `continue_live: true` in `config.yaml` and `two-config.yaml`; source pin of the start-retry gate in `driver_lidar.cc`; the fatal-error reconnect is gone since the 2026-08-31 vendor sync (upstream `18c5c93`; the old assertion failed on main from then on and was wrong, not the code) and UDP data recovers in the SDK's read loop ("recover from network timeout"); shipped configs use UDP data | `python3 -m pytest tests/test_continue_live.py -v` | < 1 s | Python 3.12, pytest, pyyaml, SDK submodule |
| `tests/legacy/` (14 tests) | unit + regression | Legacy mode = the released image: `entrypoint.sh` and `entrypoint-unified.sh` are the v1.0.8-packets files plus the opt-in block only; every fleet environment/command of `cases/*.env` (resort-retail, yard-site, flood2, airport, plant1 x2, dual flags, metrolla-edge's reference template, the switch at 0/empty) gives the golden recorded from the released entrypoint (exec'd argv and environment, output, the YAML `start_with_config.py` loads) | `python3 -m pytest tests/legacy -v` | < 1 s | Python, pytest |
| `tests/unit/` (environment contract, vendor contract, synthetic sensor) | unit | `docker/seyond_env.py`: every accepted value and refusal, `run`/`check`/`selfcheck`; every rendered key is read by `yaml_tools.hpp` with that type, defaults equal the vendor `config.yaml` except the relative topics, node name/paths/Dockerfile layout; `fake_seyond.py` packets against the SDK's packed structs and CRC32C | `python3 -m pytest tests/unit -v` | < 1 s | Python, pytest, pyyaml, SDK submodule |
| `tests/unit/test_smoke_node.py` | unit | Graph observer accepts only the exact node name, waits for delayed discovery, bounds missing/wrong-node failures and reports the last graph; graph errors propagate | `python3 -m pytest tests/unit/test_smoke_node.py -v` | < 1 s | Python, pytest; mocked ROS and clock |
| `tests/regression/` (goldens + past bugs) | regression | `check` output and refusal messages for `cases/*.env` against `golden/*.golden` (`REGEN=1` to regenerate); past bugs in contract mode: continue_live/UDP for every accepted environment, DUAL_LIDAR true refused, the node gets the written file, images without the contract refuse the switch | `python3 -m pytest tests/regression -v` | < 1 s | Python, pytest, pyyaml |
| `docker/smoke/smoke.sh IMAGE [OLD_IMAGE]` | smoke | Builds on the image: image facts; legacy fleet environments in the image (and against `seyond_drivers:v1.0.8-packets` when the run can pull it); contract `check`/refusals equal the goldens; a contract run as uid 97000, read-only root, tmpfs `/tmp`, cap-drop ALL with no sensor (keeps retrying, in the ROS graph, clean SIGTERM); a synthetic Robin-W `.inno_pc` (`fake_seyond.py`) replayed through `seyond_node` from the contract's config (only the file source added): 3 PointCloud2 on `/west/iv_points`, every point 10 m, stamps on the synthetic frames | CI job `smoke` | ~30-60 min cold | Docker |

The no-sensor graph check uses `check_node.py` with one persistent DDS participant, polling for
exactly `/east/seyond` for up to 60 s. On timeout it reports the last observed nodes and fails.
A 65 s process limit (with a 5 s kill grace) also bounds a stuck observer. The initial 15 s
wait still checks driver liveness; it cannot warm up discovery for a newly created observer.

No Seyond recording exists in Metrolla's repos, so the packets are synthetic, written from the
SDK's packet definitions. On 2026-11-21 they were also checked locally (not in CI): every packet
passed the SDK's `InnoDataPacketUtils::check_data_packet`, and `seyond_node` built with colcon in a
conda RoboStack Jazzy environment (not Ubuntu's packages, not the image) replayed them from a
contract-rendered config: 2,000 points per cloud at 10.000 m on `/west/iv_points`, frame
`seyond_west`. Not exercised: a real sensor, the live TCP/UDP session with a sensor, PTP time,
the -hb and -unified images (built by their workflows only), arm64 (built, never run).

Run everything CI runs, locally:

```bash
python3 -m pytest tests/ -v -ra
```

History: before `feat/dualdriver` merged, **5 passed, 2 skipped, 12 xfailed**; after (main
since 2026-08-31) 19 tests, of which `test_fatal_error_reconnect_gated_on_continue_live` failed
until C9 replaced it. Expected with C9 (`feat/driver-contract`): **159 passed** (no skip, no
xfail), `tests/regression` and `tests/legacy` included.

## Outcome semantics

| Outcome | Meaning | Effect |
|---|---|---|
| **PASS** | All assertions held. | Gate open. |
| **FAIL** | An assertion did not hold — a real regression (e.g. the always-true bug class reappearing, the topic typo coming back, `continue_live` flipped off). | **Blocks release.** The failure message names the field incident the assertion guards. |
| **SKIP** | A declared precondition was not met, with an explicit reason (here: `two-config.yaml` absent pre-v1.0.7-dual). | Always reported, **never silent**. |
| **ABORT** | Could-not-verify: the environment failed (interpreter, checkout, pip install) before assertions ran. | **Blocks like FAIL**, but is an *environment* failure — fix the environment, do not "fix" the code. |

Strict-xfail note: xfails in this suite are **conditional and strict** — the
condition is computed from the tree itself (fix present or not), so an
XFAIL here always means "known-missing fix, tracked to v1.0.7-dual", never
"mystery failure we silenced". The FAIL/ABORT split exists because of the
standing verify-don't-guess rule: a suite that could not run is unverified,
and unverified must never be reported as healthy.

## Where these tests run, and where the results go

**Stored** in this repo, beside the code they test — `tests/`. There is no central test tree: packages that are vendored into the `datablade` monorepo carry their tests with them, and the standalone repo stays the source of truth.

**Run** in these environments:

| Environment | What runs | When |
|---|---|---|
| Developer machine | Any suite in the inventory above | On demand — commands in this file and in `AGENTS.md` |
| CI — GitHub `ubuntu-latest` hosted runner | `unit (launch/config contract tests)`, `manifest-lint` | Every push and pull request to `main` and `feat/**` |

**Results** are stored by GitHub Actions — there is no separate test-results database:

| Output | Where | Retention |
|---|---|---|
| Full logs, per-test output, job timings | The Actions run page for this repo | 90 days (org default) |
| Job summaries | `$GITHUB_STEP_SUMMARY` on the run page | With the run |
| Test code and this inventory | This repo, in git | Permanent |

Results expire with the log retention and nothing aggregates history yet — flake rate, duration drift, or how long an xfail has been open. That is Phase 6 of the program, not built yet.

Org-wide map of every gated repo and environment: [`infra/REGRESSION-GATES.md`](https://github.com/Metrolla-Inc/infra/blob/master/REGRESSION-GATES.md).

## Adding a test

1. Put the file in `tests/`, named `test_*.py`.
2. Assert a value — never "it did not raise". If the behaviour you are pinning
   is a silent fallback, assert the fallback counter directly.
3. Add a row to the inventory above: what it asserts, its outcome rules, its
   runtime dependencies. **`manifest-lint` fails the build if you skip this.**
4. Run it: `python3 -m pytest tests/ -q`

Rules that apply to every test here:

- **No wall-clock or absolute monotonic timestamps** — `time.monotonic()` is
  seconds since boot, so a literal that works locally fails on a fresh runner.
  Always compute relative to a value captured inside the test.
- **No `sleep`, no network, no DDS, no database** in a unit test. Anything that
  needs those belongs in the replay smoke.
- **No dependence on ambient state** (a sourced workspace, a running node, a
  leftover file, another test). Tests must pass in any order.
- **Never `except: pass`** — swallowing the failure is how the original bug
  survived.
- **Frame-counted thresholds are rate-dependent**: a window in frames halves at
  10 Hz versus 5 Hz. Parametrise by rate.
- **A pass with zero rows processed is a failure** — assert the count.
- **Verify the test fails** when you break the behaviour deliberately, once,
  before opening the PR.

Pinning a bug you are not fixing yet — use a **strict xfail**, never a skip or a
deletion: `@pytest.mark.xfail(strict=True, reason="…")`. It reports every run and **turns red when the bug is fixed**,
which forces the test to be promoted. Record the reason in the inventory.

Changing a topic, message type or QoS? Update this repo's declaration under
`infra/contracts/decls/` in the same PR — the chain check validates it.

Full guidelines: [`infra/REGRESSION-GATES.md` §6](https://github.com/Metrolla-Inc/infra/blob/master/REGRESSION-GATES.md#6-writing-a-new-test--guidelines).

#!/usr/bin/env python3
# Tests for vendor_watch.py (Metrolla backlog C22). Identical copy in every fork.
# VENDOR_WATCH_VERSION = "1.0.0"
#
#   python3 -m unittest -v test_vendor_watch.Unit          throwaway local git repos
#   python3 -m unittest -v test_vendor_watch.Regression    golden issue title/body
#   VENDOR_WATCH_SMOKE=1 python3 -m unittest -v test_vendor_watch.Smoke
#                                                          real vendor, dry run, no issue
# Regenerate goldens after an intended change: VENDOR_WATCH_UPDATE_GOLDEN=1 (Regression).
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import vendor_watch as vw  # noqa: E402

GOLDEN = os.path.join(HERE, "golden")
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))

ENV = dict(os.environ, GIT_AUTHOR_NAME="t", GIT_AUTHOR_EMAIL="t@example.invalid",
           GIT_COMMITTER_NAME="t", GIT_COMMITTER_EMAIL="t@example.invalid",
           GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM="1", LC_ALL="C")


def sh(args, cwd, when=None):
    env = dict(ENV)
    if when is not None:
        stamp = "%d +0000" % (1700000000 + when * 86400)
        env["GIT_AUTHOR_DATE"] = env["GIT_COMMITTER_DATE"] = stamp
    out = subprocess.run(["git"] + args, cwd=cwd, env=env, capture_output=True, text=True)
    if out.returncode != 0:
        raise AssertionError("git %s failed: %s" % (" ".join(args), out.stderr))
    return out.stdout.strip()


class Vendor:
    """A throwaway vendor repository with commits on fixed days."""

    def __init__(self, root):
        self.path = os.path.join(root, "vendor")
        os.makedirs(self.path)
        sh(["init", "-q", "-b", "main"], self.path)
        self.day = 0

    def commit(self, msg):
        self.day += 1
        with open(os.path.join(self.path, "f.txt"), "a") as fh:
            fh.write(msg + "\n")
        sh(["add", "f.txt"], self.path)
        sh(["commit", "-q", "-m", msg], self.path, when=self.day)
        return sh(["rev-parse", "HEAD"], self.path)

    def tag(self, name, rev="HEAD"):
        sh(["tag", "-a", "-m", name, name, rev], self.path, when=self.day)


def entry(url, base, tags=("v*",), branch="main"):
    return {"id": "drv", "url": url, "branch": branch, "tags": list(tags),
            "base": {"commit": base, "how": "test"}}


class Unit(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = self.tmp.name
        self.v = Vendor(self.root)
        self.work = os.path.join(self.root, "work")
        os.makedirs(self.work)

    def tearDown(self):
        self.tmp.cleanup()

    def compare(self, e, repo_dir=None):
        work = tempfile.mkdtemp(dir=self.work)
        return vw.compare_vendor(e, repo_dir or self.root, work)

    def test_up_to_date(self):
        self.v.commit("a")
        self.v.tag("v1.0")
        tip = self.v.commit("b")
        r = self.compare(entry(self.v.path, tip))
        self.assertEqual(r["state"], "up-to-date")
        self.assertEqual(r["commits_behind"], 0)
        self.assertEqual(r["new_tags"], [])
        self.assertEqual(r["compare_url"], "")

    def test_behind_by_commits(self):
        base = self.v.commit("a")
        self.v.commit("b")
        tip = self.v.commit("c")
        r = self.compare(entry(self.v.path, base))
        self.assertEqual(r["state"], "behind")
        self.assertEqual(r["commits_behind"], 2)
        self.assertEqual(r["tip"], tip)
        self.assertEqual([c[2] for c in r["recent"]], ["c", "b"])

    def test_behind_by_tags_on_branch(self):
        base = self.v.commit("a")
        self.v.commit("b")
        self.v.tag("v2.0")
        self.v.tag("other-1")  # does not match the pattern
        r = self.compare(entry(self.v.path, base))
        self.assertEqual([t["name"] for t in r["new_tags"]], ["v2.0"])
        self.assertEqual(r["state"], "behind")

    def test_behind_by_tag_on_release_branch_only(self):
        self.v.commit("a")
        sh(["checkout", "-q", "-b", "release"], self.v.path)
        self.v.commit("old fix")
        self.v.tag("v0.9")  # older than the base: not new
        sh(["checkout", "-q", "main"], self.v.path)
        base = self.v.commit("b")
        sh(["checkout", "-q", "release"], self.v.path)
        self.v.commit("new fix")
        self.v.tag("v1.1")  # newer than the base, not on main: new
        sh(["checkout", "-q", "main"], self.v.path)
        r = self.compare(entry(self.v.path, base))
        self.assertEqual(r["commits_behind"], 0)
        self.assertEqual([t["name"] for t in r["new_tags"]], ["v1.1"])
        self.assertEqual(r["state"], "behind")

    def test_no_tag_patterns_means_no_tag_check(self):
        base = self.v.commit("a")
        self.v.commit("b")
        self.v.tag("v2.0")
        r = self.compare(entry(self.v.path, base, tags=()))
        self.assertEqual(r["new_tags"], [])
        self.assertEqual(r["commits_behind"], 1)

    def test_diverged_when_vendor_rewrites_history(self):
        self.v.commit("a")
        base = self.v.commit("b")
        self.v.tag("keep-b")  # keeps the old commit fetchable
        sh(["reset", "-q", "--hard", "HEAD~1"], self.v.path)
        tip = self.v.commit("b rewritten")
        r = self.compare(entry(self.v.path, base, tags=()))
        self.assertEqual(r["state"], "diverged")
        self.assertEqual(r["fork_only"], 1)
        self.assertEqual(r["commits_behind"], 1)
        self.assertEqual(r["tip"], tip)
        self.assertIn("base is not on main", vw.summary_line(r))

    def test_base_missing(self):
        self.v.commit("a")
        r = self.compare(entry(self.v.path, "0" * 40))
        self.assertEqual(r["state"], "base-missing")
        self.assertTrue(vw.needs_issue([r]))

    def test_vendor_unreachable(self):
        r = self.compare(entry(os.path.join(self.root, "nope"), "1" * 40))
        self.assertEqual(r["state"], "unreachable")
        self.assertTrue(r["error"])
        self.assertFalse(vw.needs_issue([r]))

    def test_base_from_submodule_gitlink(self):
        self.v.commit("a")
        pinned = self.v.commit("b")
        self.v.commit("c")
        fork = os.path.join(self.root, "fork")
        os.makedirs(fork)
        sh(["init", "-q", "-b", "main"], fork)
        sh(["update-index", "--add", "--cacheinfo", "160000,%s,sdk" % pinned], fork)
        sh(["commit", "-q", "-m", "pin"], fork, when=1)
        e = {"id": "sdk", "url": self.v.path, "branch": "main", "tags": [],
             "base": {"submodule": "sdk"}}
        r = self.compare(e, repo_dir=fork)
        self.assertEqual(r["base"], pinned)
        self.assertEqual(r["base_source"], "submodule sdk at HEAD")
        self.assertEqual(r["commits_behind"], 1)
        e["base"] = {"submodule": "missing"}
        with self.assertRaises(vw.ConfigError):
            self.compare(e, repo_dir=fork)

    def test_github_compare_url(self):
        self.assertEqual(vw.github_slug("https://github.com/Lslidar/Lslidar_ROS2_driver.git"),
                         "Lslidar/Lslidar_ROS2_driver")
        self.assertIsNone(vw.github_slug("/tmp/x"))

    def test_config_validation(self):
        good = {"schema": 1, "vendors": [entry("u", "a" * 40)]}
        bad_cases = [
            {"schema": 2, "vendors": []},
            {"schema": 1, "vendors": [entry("u", "abc")]},
            {"schema": 1, "vendors": [entry("u", "a" * 40), entry("u", "b" * 40)]},
            {"schema": 1, "vendors": [{"id": "x", "url": "u", "branch": "m",
                                       "base": {"commit": "a" * 40, "submodule": "p"}}]},
        ]
        path = os.path.join(self.root, "c.json")
        with open(path, "w") as fh:
            json.dump(good, fh)
        self.assertEqual(vw.load_config(path)["not_checked"], [])
        for case in bad_cases:
            with open(path, "w") as fh:
                json.dump(case, fh)
            with self.assertRaises(vw.ConfigError):
                vw.load_config(path)

    def test_main_dry_run_writes_nothing_and_exit_codes(self):
        base = self.v.commit("a")
        self.v.commit("b")
        path = os.path.join(self.root, "c.json")
        with open(path, "w") as fh:
            json.dump({"schema": 1, "vendors": [entry(self.v.path, base)]}, fh)

        class NoNetwork:
            def request(self, *a):
                raise AssertionError("dry run must not call GitHub")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = vw.main(["--config", path, "--repo-dir", self.root, "--dry-run"],
                           gh=NoNetwork())
        self.assertEqual(code, 0)
        self.assertIn("state behind: 1 commit behind", buf.getvalue())
        self.assertIn("dry run: would open or update", buf.getvalue())
        with open(path, "w") as fh:
            json.dump({"schema": 1, "vendors": [entry(os.path.join(self.root, "nope"),
                                                      base)]}, fh)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(vw.main(["--config", path, "--repo-dir", self.root]), 2)


def result(state, **kw):
    r = {"id": "drv", "url": "https://github.com/v/drv.git", "branch": "main", "fork_ref": "",
         "how": "", "base": "a" * 40, "base_source": "recorded in config", "tip": "b" * 40,
         "state": state, "commits_behind": 0, "fork_only": 0, "new_tags": [], "recent": [],
         "compare_url": "", "error": ""}
    r.update(kw)
    return r


class FakeGitHub:
    def __init__(self, issues=(), has_issues=True):
        self.repo = "o/r"
        self.issues = [dict(i) for i in issues]
        self.has_issues = has_issues
        self.calls = []

    def request(self, method, path, body=None):
        self.calls.append((method, path.split("?")[0], body))
        if method == "GET" and path == "/repos/o/r":
            return 200, {"has_issues": self.has_issues}
        if method == "GET" and path.startswith("/repos/o/r/issues"):
            return 200, [i for i in self.issues if i.get("state", "open") == "open"]
        if method == "POST" and path == "/repos/o/r/labels":
            return 201, {}
        if method == "POST" and path == "/repos/o/r/issues":
            issue = dict(body, number=len(self.issues) + 1, state="open")
            self.issues.append(issue)
            return 201, issue
        if method == "PATCH":
            num = int(path.rsplit("/", 1)[1])
            issue = [i for i in self.issues if i["number"] == num][0]
            issue.update(body)
            return 200, issue
        if method == "POST" and path.endswith("/comments"):
            return 201, {}
        return 404, {"message": "unexpected %s %s" % (method, path)}

    def writes(self):
        return [(m, p) for m, p, _ in self.calls if m != "GET"]


class UnitIssue(unittest.TestCase):
    """Issue decisions against a fake GitHub (part of the unit tier)."""

    def run_sync(self, gh, results):
        body = vw.render_body("o/r", results, [])
        logs = []
        code = vw.sync_issue(gh, results, body, log=logs.append)
        return code, body, logs

    def test_create_when_behind(self):
        gh = FakeGitHub()
        code, body, _ = self.run_sync(gh, [result("behind", commits_behind=3)])
        self.assertEqual(code, 0)
        self.assertEqual(gh.writes(), [("POST", "/repos/o/r/labels"),
                                       ("POST", "/repos/o/r/issues")])
        self.assertEqual(gh.issues[0]["title"], vw.TITLE)
        self.assertEqual(gh.issues[0]["labels"], [vw.LABEL])

    def test_update_instead_of_duplicate(self):
        old = {"number": 7, "title": vw.TITLE, "body": "old", "state": "open"}
        other = {"number": 8, "title": "something else", "body": "", "state": "open"}
        gh = FakeGitHub([old, other])
        code, body, _ = self.run_sync(gh, [result("behind", commits_behind=3)])
        self.assertEqual(gh.writes(), [("PATCH", "/repos/o/r/issues/7")])
        self.assertEqual(gh.issues[0]["body"], body)

    def test_no_write_when_issue_is_current(self):
        results = [result("behind", commits_behind=3)]
        body = vw.render_body("o/r", results, [])
        gh = FakeGitHub([{"number": 7, "title": vw.TITLE, "body": body}])
        self.run_sync(gh, results)
        self.assertEqual(gh.writes(), [])

    def test_close_when_caught_up(self):
        gh = FakeGitHub([{"number": 7, "title": vw.TITLE, "body": "old"}])
        self.run_sync(gh, [result("up-to-date", tip="a" * 40)])
        self.assertEqual(gh.writes(), [("POST", "/repos/o/r/issues/7/comments"),
                                       ("PATCH", "/repos/o/r/issues/7")])
        self.assertEqual(gh.issues[0]["state"], "closed")

    def test_unreachable_never_closes(self):
        gh = FakeGitHub([{"number": 7, "title": vw.TITLE, "body": "old"}])
        self.run_sync(gh, [result("up-to-date"), result("unreachable", tip="", error="x")])
        self.assertEqual(gh.writes(), [])

    def test_nothing_when_up_to_date_and_no_issue(self):
        gh = FakeGitHub()
        self.run_sync(gh, [result("up-to-date")])
        self.assertEqual(gh.writes(), [])

    def test_issues_disabled_fails_only_when_behind(self):
        code, _, logs = self.run_sync(FakeGitHub(has_issues=False),
                                      [result("behind", commits_behind=1)])
        self.assertEqual(code, 3)
        self.assertIn("Issues are disabled", logs[0])
        code, _, _ = self.run_sync(FakeGitHub(has_issues=False), [result("up-to-date")])
        self.assertEqual(code, 0)


GOLDEN_RESULTS = [
    result("behind", id="vendor-driver", url="https://github.com/Vendor/driver.git",
           branch="master", fork_ref="main", how="merge-base of main and vendor master",
           commits_behind=12, compare_url="https://github.com/Vendor/driver/compare/"
           + "a" * 40 + "..." + "b" * 40,
           new_tags=[{"name": "v2.1.0", "sha": "c" * 40, "date": "2026-10-01"},
                     {"name": "v2.0.0", "sha": "d" * 40, "date": "2026-04-01"}],
           recent=[["bbbbbbb", "2026-10-27", "Merge pull request #9 | fix"],
                   ["ccccccc", "2026-10-01", "Release 2.1.0"]]),
    result("up-to-date", id="vendor-sdk", url="https://github.com/Vendor/sdk.git",
           base="e" * 40, tip="e" * 40, base_source="submodule sdk at HEAD"),
    result("diverged", id="vendor-tools", url="https://github.com/Vendor/tools.git",
           commits_behind=1, fork_only=2),
]
GOLDEN_NOT_CHECKED = [{"name": "Vendor firmware SDK", "reason": "not public"}]


class Regression(unittest.TestCase):
    def check(self, name, text):
        path = os.path.join(GOLDEN, name)
        if os.environ.get("VENDOR_WATCH_UPDATE_GOLDEN") == "1":
            os.makedirs(GOLDEN, exist_ok=True)
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(text)
        with open(path, encoding="utf-8") as fh:
            self.assertEqual(text, fh.read(), "golden %s differs" % name)

    def test_title(self):
        self.assertEqual(vw.render_title(), "Vendor watch: this fork is behind its vendor")

    def test_issue_body(self):
        self.check("issue_body.md",
                   vw.render_body("Metrolla-Inc/example", GOLDEN_RESULTS, GOLDEN_NOT_CHECKED))

    def test_close_comment(self):
        up = [result("up-to-date", id="vendor-driver", tip="a" * 40)]
        self.check("close_comment.md", vw.render_close_comment(up))

    def test_report(self):
        self.check("report.txt", vw.render_report(GOLDEN_RESULTS, GOLDEN_NOT_CHECKED))


@unittest.skipUnless(os.environ.get("VENDOR_WATCH_SMOKE") == "1",
                     "set VENDOR_WATCH_SMOKE=1 (needs network)")
class Smoke(unittest.TestCase):
    """The real script, real vendor remotes, this repo's recorded bases, dry run."""

    def test_dry_run_against_real_vendors(self):
        config = os.path.join(REPO_ROOT, ".github", "vendor-watch.json")
        env = {k: v for k, v in os.environ.items() if k not in ("GITHUB_TOKEN", "GH_TOKEN")}
        env.pop("GITHUB_STEP_SUMMARY", None)
        proc = subprocess.run([sys.executable, os.path.join(HERE, "vendor_watch.py"),
                               "--config", config, "--repo-dir", REPO_ROOT, "--dry-run"],
                              env=env, capture_output=True, text=True, timeout=1800)
        sys.stdout.write(proc.stdout)
        sys.stderr.write(proc.stderr)
        self.assertEqual(proc.returncode, 0)
        with open(config, encoding="utf-8") as fh:
            cfg = json.load(fh)
        for v in cfg["vendors"]:
            self.assertIn("%s: %s @ %s" % (v["id"], v["url"], v["branch"]), proc.stdout)
        self.assertNotIn("state unreachable", proc.stdout)
        self.assertNotIn("state base-missing", proc.stdout)
        self.assertIn("dry run:", proc.stdout)
        self.assertNotIn("issue action:", proc.stdout)


if __name__ == "__main__":
    unittest.main()

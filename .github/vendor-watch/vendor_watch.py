#!/usr/bin/env python3
# vendor-watch: compare this fork with its vendor repositories (Metrolla backlog C22).
# VENDOR_WATCH_VERSION = "1.0.0"
#
# This file is copied, byte for byte, into every Metrolla driver fork that has a public
# vendor. Do not edit one copy: change all copies together and bump the version above.
# Only the per-repo file .github/vendor-watch.json differs between repos.
"""Report when a fork falls behind its vendor, and keep one GitHub issue about it.

Config (.github/vendor-watch.json), one entry per vendor repository the fork contains:

    {
      "schema": 1,
      "vendors": [
        {
          "id": "short name used in the report",
          "url": "https://github.com/<owner>/<repo>.git",      public; no token is used
          "branch": "vendor branch to follow",
          "tags": ["v*"],                                       fnmatch patterns, [] = none
          "fork_ref": "the fork branch the base describes",    informational
          "base": {"commit": "<sha>", "how": "how it was established"}
              or  {"submodule": "<path>", "how": "..."}         gitlink at HEAD of the checkout
        }
      ],
      "not_checked": [{"name": "...", "reason": "..."}]
    }

"Behind" means: the vendor branch has commits the recorded base does not contain, or a
tag matching "tags" exists that the base does not contain and that is either on the
vendor branch or (for release branches) committed later than the base.
States: up-to-date, behind, diverged (the base is not on the vendor branch: vendor
rewrote history or the wrong branch is tracked), base-missing (the vendor does not have
the base commit), unreachable (fetch failed).

The base is never moved by this script. After taking (or deliberately declining) vendor
changes, a person updates "base" in the config, with "how" saying what was done.

Issue handling (only with --issue; the default is a dry run that writes nothing):
one open issue, found by label "vendor-watch" and the fixed title. It is created when a
vendor needs attention, its body is replaced when the comparison changes, and it is
closed with a comment when every vendor is up to date. An unreachable vendor never closes
the issue. If Issues are disabled on the repository the run fails instead (exit 3), so a
fork that falls behind is still visible as a red run.

Exit codes: 0 compared, 1 config error, 2 a vendor was unreachable, 3 an issue is needed
but Issues are disabled, 4 GitHub API error.
"""

import argparse
import fnmatch
import json
import os
import re
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request

VERSION = "1.0.0"
LABEL = "vendor-watch"
TITLE = "Vendor watch: this fork is behind its vendor"
MARKER = "<!-- vendor-watch -->"
NEEDS_ATTENTION = ("behind", "diverged", "base-missing")
MAX_TAGS = 30
MAX_COMMITS = 10
FETCH_TIMEOUT = 900


class ConfigError(Exception):
    pass


# ---------------------------------------------------------------- git helpers

def git(args, cwd=None, check=True, timeout=300):
    env = dict(os.environ, GIT_TERMINAL_PROMPT="0", GIT_ASKPASS="true", LC_ALL="C")
    proc = subprocess.run(["git"] + args, cwd=cwd, env=env, capture_output=True,
                          text=True, timeout=timeout)
    if check and proc.returncode != 0:
        raise subprocess.CalledProcessError(proc.returncode, ["git"] + args,
                                            proc.stdout, proc.stderr)
    return proc


def is_ancestor(gitdir, a, b):
    return git(["merge-base", "--is-ancestor", a, b], cwd=gitdir, check=False).returncode == 0


def commit_exists(gitdir, sha):
    return git(["cat-file", "-e", sha + "^{commit}"], cwd=gitdir, check=False).returncode == 0


def commit_time(gitdir, rev):
    return int(git(["log", "-1", "--format=%ct", rev], cwd=gitdir).stdout.strip())


# ---------------------------------------------------------------- config

SHA_RE = re.compile(r"^[0-9a-f]{40}$")


def load_config(path):
    try:
        with open(path, encoding="utf-8") as fh:
            cfg = json.load(fh)
    except (OSError, ValueError) as exc:
        raise ConfigError("cannot read %s: %s" % (path, exc))
    if cfg.get("schema") != 1:
        raise ConfigError("schema must be 1")
    vendors = cfg.get("vendors")
    if not isinstance(vendors, list):
        raise ConfigError("vendors must be a list")
    seen = set()
    for v in vendors:
        for key in ("id", "url", "branch", "base"):
            if key not in v:
                raise ConfigError("vendor entry without %r: %r" % (key, v))
        if v["id"] in seen:
            raise ConfigError("duplicate vendor id %r" % v["id"])
        seen.add(v["id"])
        base = v["base"]
        if ("commit" in base) == ("submodule" in base):
            raise ConfigError("%s: base needs exactly one of commit or submodule" % v["id"])
        if "commit" in base and not SHA_RE.match(base["commit"]):
            raise ConfigError("%s: base.commit must be a full 40-character sha" % v["id"])
        if not isinstance(v.get("tags", []), list):
            raise ConfigError("%s: tags must be a list of patterns" % v["id"])
    cfg.setdefault("not_checked", [])
    return cfg


def resolve_base(vendor, repo_dir):
    """Return (sha, source text) for the vendor commit the fork is based on."""
    base = vendor["base"]
    if "commit" in base:
        return base["commit"], "recorded in config"
    path = base["submodule"]
    out = git(["ls-tree", "HEAD", "--", path], cwd=repo_dir).stdout.split()
    if len(out) < 3 or out[1] != "commit":
        raise ConfigError("%s: %s is not a submodule at HEAD" % (vendor["id"], path))
    return out[2], "submodule %s at HEAD" % path


# ---------------------------------------------------------------- comparison

def github_slug(url):
    m = re.match(r"^https://github\.com/([^/]+)/([^/]+?)(?:\.git)?/?$", url)
    return (m.group(1) + "/" + m.group(2)) if m else None


def fetch_vendor(url, branch, base, gitdir):
    """Fetch the vendor branch and tags (commits only) into a scratch bare repo."""
    git(["init", "--quiet", "--bare", gitdir])
    git(["remote", "add", "vendor", url], cwd=gitdir)
    git(["config", "remote.vendor.promisor", "true"], cwd=gitdir)
    git(["config", "remote.vendor.partialclonefilter", "tree:0"], cwd=gitdir)
    git(["fetch", "--quiet", "--no-tags", "--filter=tree:0", "vendor",
         "+refs/heads/%s:refs/watch/branch" % branch,
         "+refs/tags/*:refs/watch/tags/*"], cwd=gitdir, timeout=FETCH_TIMEOUT)
    if not commit_exists(gitdir, base):
        # The base may sit on a branch that is gone; GitHub still serves reachable shas.
        git(["fetch", "--quiet", "--no-tags", "--filter=tree:0", "vendor", base],
            cwd=gitdir, check=False, timeout=FETCH_TIMEOUT)


def new_tags(gitdir, base, tip, patterns):
    if not patterns:
        return []
    out = git(["for-each-ref", "--no-merged=" + base,
               "--format=%(refname:strip=3)", "refs/watch/tags"], cwd=gitdir).stdout
    base_time = commit_time(gitdir, base)
    found = []
    for name in out.split():
        if not any(fnmatch.fnmatchcase(name, p) for p in patterns):
            continue
        ref = "refs/watch/tags/%s^{commit}" % name
        peeled = git(["rev-parse", "--verify", "--quiet", ref], cwd=gitdir, check=False)
        if peeled.returncode != 0:
            continue  # tag of a tree or blob
        sha = peeled.stdout.strip()
        when = commit_time(gitdir, sha)
        if is_ancestor(gitdir, sha, tip) or when > base_time:
            date = git(["log", "-1", "--format=%cs", sha], cwd=gitdir).stdout.strip()
            found.append({"name": name, "sha": sha, "date": date, "time": when})
    found.sort(key=lambda t: (-t["time"], t["name"]))
    for t in found:
        del t["time"]
    return found


def compare_vendor(vendor, repo_dir, workdir):
    res = {"id": vendor["id"], "url": vendor["url"], "branch": vendor["branch"],
           "fork_ref": vendor.get("fork_ref", ""), "how": vendor["base"].get("how", ""),
           "base": "", "base_source": "", "tip": "", "state": "", "commits_behind": 0,
           "fork_only": 0, "new_tags": [], "recent": [], "compare_url": "", "error": ""}
    base, source = resolve_base(vendor, repo_dir)
    res["base"], res["base_source"] = base, source
    gitdir = os.path.join(workdir, re.sub(r"[^A-Za-z0-9_.-]", "_", vendor["id"]) + ".git")
    try:
        fetch_vendor(vendor["url"], vendor["branch"], base, gitdir)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        res["state"] = "unreachable"
        err = getattr(exc, "stderr", "") or str(exc)
        res["error"] = err.strip().splitlines()[-1] if err.strip() else str(exc)
        return res
    tip = git(["rev-parse", "refs/watch/branch"], cwd=gitdir).stdout.strip()
    res["tip"] = tip
    if not commit_exists(gitdir, base):
        res["state"] = "base-missing"
        return res
    res["commits_behind"] = int(git(["rev-list", "--count", base + ".." + tip],
                                    cwd=gitdir).stdout)
    res["new_tags"] = new_tags(gitdir, base, tip, vendor.get("tags", []))
    log = git(["log", "--format=%h%x09%cs%x09%s", "-n", str(MAX_COMMITS), base + ".." + tip],
              cwd=gitdir).stdout
    res["recent"] = [line.split("\t", 2) for line in log.splitlines() if line]
    slug = github_slug(vendor["url"])
    if slug and base != tip:
        res["compare_url"] = "https://github.com/%s/compare/%s...%s" % (slug, base, tip)
    if not is_ancestor(gitdir, base, tip):
        res["state"] = "diverged"
        res["fork_only"] = int(git(["rev-list", "--count", tip + ".." + base],
                                   cwd=gitdir).stdout)
    elif res["commits_behind"] or res["new_tags"]:
        res["state"] = "behind"
    else:
        res["state"] = "up-to-date"
    return res


def compare_all(cfg, repo_dir, workdir=None):
    own = workdir is None
    tmp = tempfile.TemporaryDirectory(prefix="vendor-watch-") if own else None
    try:
        return [compare_vendor(v, repo_dir, tmp.name if own else workdir)
                for v in cfg["vendors"]]
    finally:
        if tmp:
            tmp.cleanup()


# ---------------------------------------------------------------- rendering

def short(sha):
    return sha[:7] if sha else "-"


def summary_line(r):
    if r["state"] == "unreachable":
        return "unreachable (%s)" % r["error"]
    if r["state"] == "base-missing":
        return "base %s not found in the vendor repository" % short(r["base"])
    parts = []
    if r["state"] == "diverged":
        parts.append("base is not on %s (%d base-only commits)"
                     % (r["branch"], r["fork_only"]))
    if r["commits_behind"]:
        parts.append("%d commit%s behind" % (r["commits_behind"],
                                             "" if r["commits_behind"] == 1 else "s"))
    if r["new_tags"]:
        parts.append("%d new tag%s" % (len(r["new_tags"]),
                                       "" if len(r["new_tags"]) == 1 else "s"))
    return ", ".join(parts) if parts else "up to date"


def needs_issue(results):
    return any(r["state"] in NEEDS_ATTENTION for r in results)


def render_title():
    return TITLE


def render_body(repo, results, not_checked):
    out = [MARKER, ""]
    out.append("The weekly vendor comparison found vendor changes that `%s` does not contain."
               % repo)
    out.append("")
    out.append("| Vendor | Branch | Fork base | Vendor tip | State |")
    out.append("|---|---|---|---|---|")
    for r in results:
        name = "[%s](%s)" % (r["id"], r["url"])
        state = summary_line(r)
        if r["state"] == "diverged":
            state = "diverged: " + state
        out.append("| %s | `%s` | `%s` | `%s` | %s |" % (name, r["branch"], short(r["base"]),
                                                     short(r["tip"]), state))
    for r in results:
        if r["state"] not in NEEDS_ATTENTION:
            continue
        out.append("")
        out.append("### %s" % r["id"])
        out.append("")
        out.append("- Vendor: %s, branch `%s`" % (r["url"], r["branch"]))
        base_note = r["base_source"]
        if r["fork_ref"]:
            base_note += ", describes fork branch `%s`" % r["fork_ref"]
        out.append("- Fork base: `%s` (%s)" % (r["base"], base_note))
        if r["how"]:
            out.append("- How the base was established: %s" % r["how"])
        if r["tip"]:
            out.append("- Vendor tip: `%s`" % r["tip"])
        if r["compare_url"]:
            out.append("- Compare: %s" % r["compare_url"])
        if r["new_tags"]:
            shown = r["new_tags"][:MAX_TAGS]
            tags = ", ".join("`%s` (%s)" % (t["name"], t["date"]) for t in shown)
            more = len(r["new_tags"]) - len(shown)
            out.append("- New tags: %s%s" % (tags, " and %d more" % more if more else ""))
        if r["recent"]:
            out.append("- Latest vendor commits not in the base (newest first, at most %d):"
                       % MAX_COMMITS)
            for sha, date, subject in r["recent"]:
                out.append("  - `%s` %s %s" % (sha, date, subject.replace("|", "\\|")))
    if not_checked:
        out.append("")
        out.append("Not checked: " + "; ".join("%s (%s)" % (n["name"], n["reason"])
                                               for n in not_checked) + ".")
    out.append("")
    out.append("To resolve: review the vendor changes, take them (or decide not to), then "
               "update `base` in `.github/vendor-watch.json` and say in `how` what was "
               "done. This issue is updated by the weekly run and closed when every vendor "
               "is up to date.")
    out.append("")
    out.append("_vendor_watch.py %s_" % VERSION)
    return "\n".join(out) + "\n"


def render_close_comment(results):
    lines = ["Every vendor is up to date with the recorded base, so this issue is closed:", ""]
    for r in results:
        lines.append("- %s `%s`: base `%s` = vendor tip `%s`" % (r["id"], r["branch"],
                                                                 short(r["base"]),
                                                                 short(r["tip"])))
    lines.append("")
    lines.append("_vendor_watch.py %s_" % VERSION)
    return "\n".join(lines) + "\n"


def render_report(results, not_checked):
    lines = []
    for r in results:
        lines.append("%s: %s @ %s" % (r["id"], r["url"], r["branch"]))
        lines.append("  base %s (%s)" % (r["base"] or "-", r["base_source"]))
        lines.append("  tip  %s" % (r["tip"] or "-"))
        lines.append("  state %s: %s" % (r["state"], summary_line(r)))
        if r["new_tags"]:
            lines.append("  new tags: " + " ".join(t["name"] for t in r["new_tags"]))
        if r["compare_url"]:
            lines.append("  compare: " + r["compare_url"])
    for n in not_checked:
        lines.append("%s: not checked (%s)" % (n["name"], n["reason"]))
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------- issue handling

def plan_issue_action(results, open_issue, body):
    """Pure decision: ('create'|'update'|'close'|'none', reason)."""
    if needs_issue(results):
        if open_issue is None:
            return "create", "a vendor needs attention and no issue is open"
        if (open_issue.get("body") or "").strip() != body.strip():
            return "update", "the comparison changed"
        return "none", "the open issue is current"
    if any(r["state"] == "unreachable" for r in results):
        return "none", "a vendor was unreachable, so the issue is left as it is"
    if open_issue is not None:
        return "close", "every vendor is up to date"
    return "none", "every vendor is up to date"


class GitHubError(Exception):
    pass


class GitHub:
    def __init__(self, repo, token, api="https://api.github.com"):
        self.repo, self.token, self.api = repo, token, api.rstrip("/")

    def request(self, method, path, body=None):
        url = self.api + path
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(url, data=data, method=method)
        req.add_header("Accept", "application/vnd.github+json")
        req.add_header("Authorization", "Bearer " + self.token)
        req.add_header("X-GitHub-Api-Version", "2022-11-28")
        req.add_header("User-Agent", "vendor-watch/" + VERSION)
        if data is not None:
            req.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(req, timeout=60) as resp:
                raw = resp.read()
                return resp.status, (json.loads(raw) if raw else None)
        except urllib.error.HTTPError as exc:
            raw = exc.read()
            try:
                payload = json.loads(raw) if raw else None
            except ValueError:
                payload = {"message": raw.decode(errors="replace")}
            return exc.code, payload


def call(gh, method, path, body=None, ok=(200, 201)):
    status, payload = gh.request(method, path, body)
    if status not in ok:
        msg = payload.get("message") if isinstance(payload, dict) else payload
        raise GitHubError("%s %s -> %s %s" % (method, path, status, msg))
    return payload


def find_open_issue(gh):
    issues = call(gh, "GET", "/repos/%s/issues?state=open&labels=%s&per_page=100"
                  % (gh.repo, LABEL))
    mine = [i for i in issues if "pull_request" not in i and i.get("title") == TITLE]
    mine.sort(key=lambda i: i["number"])
    return mine[0] if mine else None


def sync_issue(gh, results, body, log=print):
    """Apply the issue decision. Returns an exit code (0 or 3)."""
    repo = call(gh, "GET", "/repos/%s" % gh.repo)
    if not repo.get("has_issues", False):
        if needs_issue(results):
            log("::error::%s is behind its vendor, but Issues are disabled on this "
                "repository, so no issue can be opened. Enable Issues or read the "
                "comparison above." % gh.repo)
            return 3
        log("Issues are disabled on this repository; nothing to report.")
        return 0
    issue = find_open_issue(gh)
    action, reason = plan_issue_action(results, issue, body)
    log("issue action: %s (%s)" % (action, reason))
    if action == "create":
        status, _ = gh.request("POST", "/repos/%s/labels" % gh.repo,
                               {"name": LABEL, "color": "c5def5",
                                "description": "Fork is behind its vendor (vendor_watch.py)"})
        if status not in (201, 422):
            raise GitHubError("cannot create label %s: HTTP %s" % (LABEL, status))
        created = call(gh, "POST", "/repos/%s/issues" % gh.repo,
                       {"title": TITLE, "body": body, "labels": [LABEL]})
        log("opened issue #%s" % created["number"])
    elif action == "update":
        call(gh, "PATCH", "/repos/%s/issues/%s" % (gh.repo, issue["number"]), {"body": body})
        log("updated issue #%s" % issue["number"])
    elif action == "close":
        call(gh, "POST", "/repos/%s/issues/%s/comments" % (gh.repo, issue["number"]),
             {"body": render_close_comment(results)})
        call(gh, "PATCH", "/repos/%s/issues/%s" % (gh.repo, issue["number"]),
             {"state": "closed", "state_reason": "completed"})
        log("closed issue #%s" % issue["number"])
    return 0


# ---------------------------------------------------------------- main

def main(argv=None, gh=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=".github/vendor-watch.json")
    ap.add_argument("--repo-dir", default=".", help="checkout of the fork")
    ap.add_argument("--repo", default=os.environ.get("GITHUB_REPOSITORY", "this repository"),
                    help="owner/name, used for the issue")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true",
                      help="compare and print the issue that would be written (default)")
    mode.add_argument("--issue", action="store_true",
                      help="open, update or close the GitHub issue (needs GITHUB_TOKEN)")
    ap.add_argument("--version", action="version", version=VERSION)
    args = ap.parse_args(argv)

    try:
        cfg = load_config(args.config)
        results = compare_all(cfg, args.repo_dir)
    except ConfigError as exc:
        print("config error: %s" % exc, file=sys.stderr)
        return 1

    print("vendor_watch.py %s: %s" % (VERSION, args.repo))
    print(render_report(results, cfg["not_checked"]), end="")
    body = render_body(args.repo, results, cfg["not_checked"])
    summary = os.environ.get("GITHUB_STEP_SUMMARY")
    if summary:
        with open(summary, "a", encoding="utf-8") as fh:
            fh.write("## Vendor watch\n\n```\n%s```\n" % render_report(results,
                                                                      cfg["not_checked"]))
    code = 0
    if not args.issue:
        if needs_issue(results):
            print("dry run: would open or update the issue %r with this body:" % TITLE)
            print(body, end="")
        else:
            print("dry run: no vendor needs attention; an open issue would be closed")
    else:
        if gh is None:
            token = os.environ.get("GITHUB_TOKEN", "")
            if not token or "/" not in args.repo:
                print("--issue needs GITHUB_TOKEN and --repo owner/name", file=sys.stderr)
                return 1
            gh = GitHub(args.repo, token,
                        os.environ.get("GITHUB_API_URL", "https://api.github.com"))
        try:
            code = sync_issue(gh, results, body)
        except GitHubError as exc:
            print("::error::GitHub API: %s" % exc)
            return 4
    if any(r["state"] == "unreachable" for r in results):
        print("::error::a vendor repository could not be fetched")
        return 2
    return code


if __name__ == "__main__":
    sys.exit(main())

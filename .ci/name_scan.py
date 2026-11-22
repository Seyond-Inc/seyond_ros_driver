#!/usr/bin/env python3
"""Customer-name check: fails when a tracked file (its text or its path) contains a deny-listed name.

    python3 .ci/name_scan.py [--root DIR | --tree DIR] [--json] [--deny FILE] [--allow FILE] [PATH ...]

Exit codes: 0 no hit, 1 hit (or a bad list file), 2 never used. Standard library only.

The deny-list (.ci/name-denylist.txt) has one entry per line, '#' starts a comment:
    <name> [word]                 a plain name, may hold spaces (private repositories)
    sha256:<len>:<hex> [word]     the sha256 of the lowercased name, and its length (public repositories:
                                  the list itself names nobody; make one with --hash NAME)
Matching is case-insensitive, ignores accents, and treats runs of spaces, '-' and '_' as one '_', so
'Foo Bar', 'foo-bar' and 'FOO_BAR' all match the entry 'foo bar'. A name matches anywhere inside a word
or an identifier ('xFoobar2' matches 'foobar'); with 'word' it must be a whole run of letters (so a
short name does not match inside a longer word).

The allowlist (.ci/name-allowlist.txt) records the hits that stay, one per line:
    <path glob> | <regex or *> | <reason>
A hit on a line of a matching file is allowed when it lies inside a match of the regex (the regex
matches are blanked before the line is checked again); '*' allows every hit in the file. Paths given
on the command line limit the scan to those files (default: every file `git ls-files` lists).
Binary files: only their path, and their bytes for names of 5 or more letters (not 'word' names).
"""
import argparse
import fnmatch
import hashlib
import json
import os
import re
import subprocess
import sys
import unicodedata

SEP = re.compile(r"[\s\-_]+")
RUN = re.compile(r"[a-z0-9_]+")
LETTERS = re.compile(r"[a-z]+")


def normalise(text):
    text = unicodedata.normalize("NFKD", text)
    text = "".join(c for c in text if not unicodedata.combining(c))
    return SEP.sub("_", text.lower())


def sha(text):
    return hashlib.sha256(text.encode()).hexdigest()


class Deny:
    def __init__(self, lines):
        self.plain_sub, self.plain_word, self.hash_sub, self.hash_word = [], set(), {}, set()
        for n, raw in enumerate(lines, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            word = len(parts) > 1 and parts[-1] == "word"
            entry = " ".join(parts[:-1] if word else parts)
            if entry.startswith("sha256:"):
                bits = entry.split(":")
                if " " in entry or len(bits) != 3 or not bits[1].isdigit() or not re.fullmatch(r"[0-9a-f]{64}", bits[2]):
                    raise ValueError(f"deny-list line {n}: expected sha256:<len>:<64 hex>")
                if word:
                    self.hash_word.add(bits[2])
                else:
                    self.hash_sub.setdefault(int(bits[1]), set()).add(bits[2])
            else:
                name = normalise(entry)
                (self.plain_word.add(name) if word else self.plain_sub.append(name))
        self.sub_re = re.compile("|".join(re.escape(t) for t in sorted(self.plain_sub, key=len, reverse=True))) \
            if self.plain_sub else None
        self._cache = {}

    def __len__(self):
        return len(self.plain_sub) + len(self.plain_word) + len(self.hash_word) + \
            sum(len(v) for v in self.hash_sub.values())

    def _run_hits(self, run):
        got = self._cache.get(run)
        if got is None:
            got = []
            for length, hashes in self.hash_sub.items():
                for i in range(0, len(run) - length + 1):
                    if sha(run[i:i + length]) in hashes:
                        got.append(run[i:i + length])
            self._cache[run] = got
        return got

    def hits(self, text):
        """The deny-listed names in text (normalised spellings; a hashed name shows as found in the text)."""
        t = normalise(text)
        out = []
        if self.sub_re:
            out += self.sub_re.findall(t)
        if self.plain_word or self.hash_word:
            for w in LETTERS.findall(t):
                if w in self.plain_word or (self.hash_word and sha(w) in self.hash_word):
                    out.append(w)
        if self.hash_sub:
            for run in set(RUN.findall(t)):
                out += self._run_hits(run)
        return out

    def byte_names(self):
        return [t for t in self.plain_sub if len(t) >= 5]


def load_allow(path):
    rules = []
    if not os.path.exists(path):
        return rules
    for n, raw in enumerate(open(path, encoding="utf-8"), 1):
        line = raw.rstrip("\n")
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        parts = line.split(" | ", 2)
        if len(parts) != 3 or not parts[2].strip():
            raise ValueError(f"allowlist line {n}: expected '<path glob> | <regex or *> | <reason>'")
        glob, rx = parts[0].strip(), parts[1].strip()
        rules.append((glob, None if rx == "*" else re.compile(rx, re.I), n))
    return rules


def tracked(root):
    out = subprocess.run(["git", "-C", root, "ls-files", "-z"], check=True, capture_output=True).stdout
    return [p for p in out.decode("utf-8", "surrogateescape").split("\0") if p]


def scan(root, deny, allow, paths):
    hits, used = [], set()

    def rules_for(path):
        return [r for r in allow if fnmatch.fnmatch(path, r[0])]

    def check(path, lineno, text, rules):
        names = deny.hits(text)
        if not names:
            return
        for glob, rx, n in rules:
            if rx is None:
                used.add(n)
                return
            blanked = rx.sub(" ", text)
            if blanked != text:
                if not deny.hits(blanked):
                    used.add(n)
                    return
                text = blanked
                used.add(n)
        names = deny.hits(text)
        if names:
            hits.append({"path": path, "line": lineno, "names": sorted(set(names)), "text": text.strip()[:200]})

    for path in paths:
        rules = rules_for(path)
        check(path, 0, path, rules)          # the path itself
        full = os.path.join(root, path)
        if not os.path.isfile(full) or os.path.islink(full):
            continue
        data = open(full, "rb").read()
        if b"\0" in data[:8192]:
            low = data.lower()
            found = sorted({t for t in deny.byte_names() if t.encode() in low or t.replace("_", "-").encode() in low})
            if found and not any(r[1] is None for r in rules):
                hits.append({"path": path, "line": -1, "names": found, "text": "(binary file)"})
            elif found:
                used.update(r[2] for r in rules if r[1] is None)
            continue
        text = data.decode("utf-8", "replace")
        for i, line in enumerate(text.splitlines(), 1):
            check(path, i, line, rules)
    stale = [r for r in allow if r[2] not in used]
    return hits, stale


def self_test():
    """The matching rules, on made-up names (plain and hashed must agree)."""
    names = ["zorblax", "quuxco", "fizz yard", "qx"]
    plain = Deny(["zorblax", "quuxco", "fizz yard", "qx word"])
    hashed = Deny([f"sha256:{len(normalise(n))}:{sha(normalise(n))}" for n in names[:3]] +
                  [f"sha256:2:{sha('qx')} word"])
    cases = [("the Zorblax site", 1), ("ZORBLAX_entry", 1), ("x_zorblax2", 1), ("zórblax", 1), ("quux co", 0),
             ("QuuxCo-dev", 1), ("Fizz Yard plan", 1), ("fizz-yard-bucket", 1), ("FIZZ_YARD", 1), ("fizzyard", 0),
             ("qx = 1", 1), ("qxt", 0), ("aqx", 0), ("qx_zone", 1), ("nothing here", 0)]
    bad = 0
    for text, want in cases:
        for kind, d in (("plain", plain), ("hashed", hashed)):
            got = len(d.hits(text))
            if bool(got) != bool(want):
                print(f"self-test FAILED ({kind}): {text!r} -> {got} hit(s), want {want}")
                bad += 1
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as fh:
        fh.write("docs/* | zorblax-sdk | the SDK's package name\nsrc/x.py | * | test fixture\n")
    rules = load_allow(fh.name)
    os.unlink(fh.name)
    with tempfile.TemporaryDirectory() as d:
        os.makedirs(os.path.join(d, "docs")); os.makedirs(os.path.join(d, "src"))
        open(os.path.join(d, "docs", "a.md"), "w").write("uses zorblax-sdk\nthe zorblax site\n")
        open(os.path.join(d, "src", "x.py"), "w").write("zorblax\n")
        open(os.path.join(d, "src", "fizz-yard.bin"), "wb").write(b"\0\0zorblax")
        hits, stale = scan(d, plain, rules, ["docs/a.md", "src/x.py", "src/fizz-yard.bin"])
        got = sorted((h["path"], h["line"]) for h in hits)
        want = [("docs/a.md", 2), ("src/fizz-yard.bin", -1), ("src/fizz-yard.bin", 0)]
        if got != want or stale:
            print(f"self-test FAILED (scan): {got} != {want}, stale {stale}")
            bad += 1
    print("name scan self-test: " + ("FAILED" if bad else "ok"))
    return 1 if bad else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("paths", nargs="*")
    ap.add_argument("--root", default=".")
    ap.add_argument("--tree", metavar="DIR", help="scan every file under DIR (not a git checkout: a built package, "
                    "a docs build); the lists still default to <root>/.ci/")
    ap.add_argument("--deny", default=None, help="default <root>/.ci/name-denylist.txt")
    ap.add_argument("--allow", default=None, help="default <root>/.ci/name-allowlist.txt")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--hash", metavar="NAME", help="print the hashed deny-list entry for NAME and exit")
    ap.add_argument("--self-test", action="store_true", help="check the matching rules on made-up names and exit")
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()
    if a.hash:
        n = normalise(a.hash)
        print(f"sha256:{len(n)}:{sha(n)}")
        return 0
    root = os.path.abspath(a.root)
    try:
        deny = Deny(open(a.deny or os.path.join(root, ".ci", "name-denylist.txt"), encoding="utf-8"))
        allow = load_allow(a.allow or os.path.join(root, ".ci", "name-allowlist.txt"))
    except (OSError, ValueError) as e:
        print(f"name scan: {e}", file=sys.stderr)
        return 1
    if not len(deny):
        print("name scan: the deny-list is empty", file=sys.stderr)
        return 1
    own = {os.path.relpath(os.path.abspath(f), root) for f in
           (a.deny or os.path.join(root, ".ci", "name-denylist.txt"), a.allow or os.path.join(root, ".ci", "name-allowlist.txt"))}
    if a.tree:
        tree = os.path.abspath(a.tree)
        paths = sorted(os.path.relpath(os.path.join(d, f), tree) for d, _, fs in os.walk(tree) for f in fs)
        hits, stale = scan(tree, deny, allow, paths)
    else:
        paths = [p for p in (a.paths or tracked(root)) if p not in own]      # the two lists are not scanned
        hits, stale = scan(root, deny, allow, paths)
    if a.json:
        print(json.dumps({"files": len(paths), "names": len(deny), "hits": hits,
                          "unused_allowlist_lines": [r[2] for r in stale]}, indent=1))
    else:
        for h in hits:
            print(f"{h['path']}:{h['line']}: {h['text']}")
        for r in stale:
            print(f"name scan: note: allowlist line {r[2]} ({r[0]}) matched nothing", file=sys.stderr)
        print(f"name scan: {len(paths)} files, {len(deny)} names, {len(hits)} hits", file=sys.stderr)
    return 1 if hits else 0


if __name__ == "__main__":
    sys.exit(main())

<!-- vendor-watch -->

The weekly vendor comparison found vendor changes that `Metrolla-Inc/example` does not contain.

| Vendor | Branch | Fork base | Vendor tip | State |
|---|---|---|---|---|
| [vendor-driver](https://github.com/Vendor/driver.git) | `master` | `aaaaaaa` | `bbbbbbb` | 12 commits behind, 2 new tags |
| [vendor-sdk](https://github.com/Vendor/sdk.git) | `main` | `eeeeeee` | `eeeeeee` | up to date |
| [vendor-tools](https://github.com/Vendor/tools.git) | `main` | `aaaaaaa` | `bbbbbbb` | diverged: base is not on main (2 base-only commits), 1 commit behind |

### vendor-driver

- Vendor: https://github.com/Vendor/driver.git, branch `master`
- Fork base: `aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa` (recorded in config, describes fork branch `main`)
- How the base was established: merge-base of main and vendor master
- Vendor tip: `bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb`
- Compare: https://github.com/Vendor/driver/compare/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa...bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
- New tags: `v2.1.0` (2026-10-01), `v2.0.0` (2026-04-01)
- Latest vendor commits not in the base (newest first, at most 10):
  - `bbbbbbb` 2026-10-27 Merge pull request #9 \| fix
  - `ccccccc` 2026-10-01 Release 2.1.0

### vendor-tools

- Vendor: https://github.com/Vendor/tools.git, branch `main`
- Fork base: `aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa` (recorded in config)
- Vendor tip: `bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb`

Not checked: Vendor firmware SDK (not public).

To resolve: review the vendor changes, take them (or decide not to), then update `base` in `.github/vendor-watch.json` and say in `how` what was done. This issue is updated by the weekly run and closed when every vendor is up to date.

_vendor_watch.py 1.0.0_

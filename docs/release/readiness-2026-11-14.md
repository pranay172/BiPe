# Release readiness (14 November 2026)

A pass over `main` at 3d193a2 (M16 merged), from a clean clone, before the
repository goes public. It follows the [29 October audit](audit-2026-10-29.md).

## Checked

- **Quick start.** On a 4-core cloud CPU, following the README: install from
  `requirements.lock` about 20 s, `python -m bipe sim --headless` 14 s, all
  eight tour checks pass. The interactive window was not opened (no display).
- **Tests.** `pytest`: 246 passed, 10 skipped, with `MUJOCO_GL=osmesa` and
  `libosmesa6` installed; without an OpenGL context the rendered tests fail. `make -C firmware test`: all pass.
- **Secrets.** None in the tree, and none in the diffs since the last audit
  (GitHub, Anthropic, AWS and Hugging Face token patterns, private-key headers,
  credential file names).
- **Name.** The commercial robot's name no longer appears in the tree.
- **Licences.** `LICENSE` (Apache-2.0), `LICENSE-CAD` (CC BY-SA 4.0),
  `LICENSE-HARDWARE` (CERN-OHL-P-2.0) hold the full texts; `NOTICE` gives the
  scope of each and the attributions.
- **Size.** No file over 15 MB. Tree 540 MB on disk; about 450 MB compressed.

## Fixed on this branch

| Finding | Fix |
|---|---|
| The GPL-3.0 blender-mcp wheel, its add-on, a Blender crash log and install logs were committed (audit item 2) | `artifacts/tooling/` removed and gitignored; `docs/blender-mcp.md` builds the server from its pinned upstream revision; `NOTICE` updated |
| `/home/pranay/...` paths and the laptop's host name in 102 tracked files (audit item 4) | Rewritten as repository-relative paths, `~` or "laptop" |
| The README, `docs/README.md` and `docs/results.md` stopped at M15 (get-up 52/60, "next: M16") | M16 results, the held-out get-up (53/60), sit and stand, current gaps and roadmap |
| `docs/demo/README.md` linked a private planning page | Link removed |
| `python -m bipe sim --help` printed the top-level help instead of the keys the README promises | `--help` now reaches the sim's parser; `tests/test_cli.py` |
| `make -C firmware` built `libbipe.so` without `src/fallen.c`, so the Python tests failed with a missing `bipe_fallen_step` after the documented firmware tests ran | `fallen.c` added to the core sources |

## Still blocking: needs a decision

- **Git history.** 487 commits, 562 MB packed. Old commits still carry the
  GPL wheel, the commercial robot's name, the personal paths and a personal
  Gmail address on 278 commits. Making this repository public publishes all
  of it. Recommended: publish a fresh, single-commit history (in a new public
  repository, or a new history in this one after keeping a private copy) and
  keep the full history private as the record.

## Nice to have

- A demo clip or GIF on the README; today it shows one still. The last cut
  (3 November) is classical-only; M15 learned-policy clips exist but are not
  cut together.
- CI: `pytest`, `make -C firmware test` and the headless tour on each push.
- Policies and manifests on the Hugging Face Hub, and the manifest schema
  (M16 phase H, not done).
- A smaller clone: about 100 MB of `summary.json` files embed full traces,
  plus 46 MB of render meshes and 61 MB of M12 traces. These could move to
  release assets.
- `bipe sim` opened on a real laptop screen (only a virtual display so far).
- `docs/history/` links to artifacts removed in the M10 cleanup; its README
  says so and points to the archive tags, which a squashed history would drop.

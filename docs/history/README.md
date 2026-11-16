# Project history (M0–M9)

These are the plans, results and topic notes from milestones 0–9. They
cover:
- the M4 articulated prototype with its electronics and firmware;
- the M5 digital release;
- the M6/M7 learning studies;
- the M8 robustness yardstick;
- the M9 five-bar redesign.

They moved here in the M10 Phase E cleanup (18 October 2026). The
current design is M10:
[`docs/milestones/10-redesign-plan.md`](../milestones/10-redesign-plan.md).

Most links in these documents point to artifacts, code and electronics
that were removed from the working tree in that cleanup.

The code, docs and electronics are preserved in git:
- tag `archive/m10-pre-cleanup`: the tree just before the cleanup;
- tag `archive/m9-final`: the M9 state.

For example, to read one removed file:

```bash
git show archive/m10-pre-cleanup:bipe/actuator_study.py
```

To check out the whole historical tree in a separate folder:

```bash
git worktree add ../bipe-archive archive/m10-pre-cleanup
```

The generated outputs of milestones 0–9 (`artifacts/milestone-*`:
traces, renders, videos) are not in this repository's history. They
were removed when the history was slimmed on 21 October 2026, so in the
tags above only the few that are still in the current tree remain
([commit-map-2026-10-21.txt](commit-map-2026-10-21.txt) explains the
rewrite). The full original history, with every output, is kept outside
the repository in the git bundle `bipe-full-history-2026-10-21.bundle`.
Its tags carry the same names, so a removed output is read from a clone
of the bundle:

```bash
git clone --mirror bipe-full-history-2026-10-21.bundle bipe-full.git
git --git-dir=bipe-full.git show archive/m10-pre-cleanup:artifacts/milestone-8/summary.json
```

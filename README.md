# sokoban-solver-01-run-20261121

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/jokelord45/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/jokelord45/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver for make.fun's "Build the best Sokoban
solver, 21 November 2026" bounty. `main.ts` answers all 5 instances of a
set in one run: it splits each level into its independent rooms (a box
pushed out of a room's door can never come back, so the rooms' optima add
up to the level's), then solves every room with A* over pushes — a state
is the boxes plus the player's reachable region — with an admissible
Hungarian heuristic on minimum-push distances pulled back from the goals,
dead-square pruning and frozen 2x2-block pruning. The search is budgeted
by work (state counts), never by the clock, so two runs write the same
bytes.

Local grade on the kit's practice set: **1,000,000 / 1,000,000** — every
instance solved in exactly its push-optimal count (13, 16, 29, 47, 60
pushes) in ~0.2 s for the whole set. The house baseline (plain-astar
1.0.0) scores 200,000 on the same set — it solves only the first
instance. On 20 further sets drawn from the kit generator with fresh
seeds (100 levels), the solver again scored 1,000,000 on every set,
deterministically, each set in under 0.5 s.

## Run it

```sh
node main.ts
```

`main.ts` reads every level from `/input/<id>/level.txt` (or
`$MAKEFUN_INPUT_DIR`) and writes `/output/<id>/solution.txt` (or
`$MAKEFUN_OUTPUT_DIR`), creating the output directories itself. Node 22+.
No dependencies, no network, no clock reads.

---

<sub>Entered on [make.fun](https://make.fun/?ref=zba7a30c&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>

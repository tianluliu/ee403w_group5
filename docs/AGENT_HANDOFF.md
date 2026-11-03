# Agent Handoff Guide

This project is being worked on through more than one AI-assisted workflow. Important engineering context must live in the repository, not only in chat history.

## Read order

1. `AGENTS.md`
2. `README.md`
3. `docs/CURRENT_STATE.md`
4. `docs/design.md`
5. `docs/interfaces.md`
6. `docs/EXPLORATION.md`
7. `docs/RUNBOOK.md`
8. `docs/tasks.md`
9. task-specific code/tests
10. `docs/requirements.md` for scope/acceptance work

## Source-of-truth priority

For implementation behavior:
1. current code/tests
2. `docs/CURRENT_STATE.md`
3. focused current docs
4. README

For formal project requirements:
1. PDR/project brief
2. `docs/requirements.md`
3. ratified team decisions recorded in the repository

If code and documentation disagree, identify the mismatch rather than guessing.

## Handoff template

```text
Branch:
Commit:
Goal:

Files changed:
- ...

Behavior changed:
- ...

Verification:
- command:
- result:

Runtime test:
- what was launched:
- observed result:

Known limitations:
- ...

Next recommended step:
- ...
```

## Agent rules

- Do not invent hardware measurements.
- Do not claim physical validation from simulation.
- Do not silently change PDR requirements.
- Preserve safety gates unless explicitly changing them.
- Run tests before calling work complete.
- Add/update tests for algorithm changes.
- Keep demo-only experiments separate from production architecture.
- Do not commit credentials, build output, generated maps/bags/logs, or local connection settings.
- Avoid unrelated refactors during focused tuning.
- Label findings as observed, simulated, inferred, or physically measured.

## Current best next software task

Benchmark the existing complex-world exploration before tuning:
- completion time
- distance traveled
- number of goals
- failed goals
- suppressed/retried goals
- coverage
- completion result

Then tune and compare against the baseline.

## Later hardware integration

1. confirm exact motor/encoder details
2. choose physical odometry source
3. implement Arduino firmware
4. implement real serial transport
5. calibrate geometry/drive constants
6. wheels-up safety test
7. low-speed physical motion
8. integrate real LiDAR + odometry + SLAM
9. integrate Nav2
10. integrate explorer

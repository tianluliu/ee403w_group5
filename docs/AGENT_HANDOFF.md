# Agent Handoff Guide

This project is being worked on through more than one AI-assisted workflow. Important engineering context must live in the repository, not only in chat history.

## Read order
1. `AGENTS.md`
2. `README.md`
3. `docs/CURRENT_STATE.md`
4. `docs/EXPLORATION.md`
5. `docs/RUNBOOK.md`
6. task-specific code/tests
7. `docs/requirements.md` for requirements
8. `docs/interfaces.md` for interfaces

## Source-of-truth priority
For current implementation/status:
1. current code/tests
2. `docs/CURRENT_STATE.md`
3. focused current docs
4. README
5. older Phase 1 status docs

PDR-derived requirements are not superseded by status docs unless the team explicitly changes scope.

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
- Keep demo-only experiments separate.
- Do not commit credentials, build output, generated maps/bags/logs, or local IP assumptions.
- Avoid unrelated refactors during focused tuning.
- Label findings as observed, simulated, inferred, or physically measured.

## Recommended next task
Benchmark the existing complex-world exploration before tuning it.

Collect:
- completion time
- distance traveled
- number of goals
- failed goals
- suppressed/retried goals
- coverage
- completion result

Then tune and compare.

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

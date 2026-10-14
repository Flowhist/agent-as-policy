---
name: embodied-agent
description: Plan, execute, or review a robot task using this repository's device tools.
---

# Embodied Agent

Work within the requested scope and existing authorization. For planning/review,
return that result; for execution, continue until the physical outcome is evidenced
or a concrete blocker requires user input. User corrections supersede old plans.

## Read Once

Use [README](../../../README.md) for current status, known dimensions, calibration
and entry points. For plan construction, scene helpers or review commands, read the
relevant section of the [plan guide](references/plan-format.md). Consult the
[tool contract](../../../docs/工具与数据契约.md) only for device-specific boundaries.
Discover adapter schemas once with `python3 scripts/validate_task_plan.py --adapter "$ADAPTER" --list-tools`; reuse them and instructions already in context. During normal
execution, do not search Git, implementation files, vendor examples or old runs to
rediscover interfaces, or edit shared control code. Debugging/maintenance is a
separate explicit task; task-specific geometry code is allowed.

Read [control.yaml](../../../config/control.yaml) once for each new task: use its
SDK rate by default (omit rate in path helpers/specs) and gripper force. Do not
hardcode historical 10%/20%, silently raise defaults, or rewrite a checked route.
Explicit task rate overrides require current user intent; YAML edits affect new
plans and subsequent gripper actions, not already checked trajectories.

## Observe and Plan

- Full physical tasks use one task directory and one `plan.json` covering the known
  remaining goal through final acceptance, advanced by `scripts/run_plan.py`.
  Initial observations/geometry may precede it; do not manually dispatch its
  actions. Read-only requests and explicit single-tool debugging need no plan.
- Read service ownership with `python3 -m scripts.hardware.sensor_client status`;
  query the SDK only while the device is idle and ownership permits it. Reuse
  resident sensors, never guess HTTP routes or start duplicate owners. Keep SDK
  and OpenCV/RealSense in separate processes and preserve all tool guards.
- Reuse known dimensions for the same objects, but acquire current poses and state.
  Inspect target, destination, occupancy, orientation and clearance in one frame.
  Refresh for changed/stale/ambiguous evidence, occlusion or action verification,
  not for a stage change or newly supplied dimensions alone.
- In one system-Python task process, share calibration and use
  `scene_batch.analyze_frame` for independent image jobs, then combine dependent
  geometry/overlays and assemble the plan with `PlanBuilder`. Inspect each evidence
  bundle once for all pending questions; local parallelism does not batch model calls.
- Preflight known motion phases with `plan_joint_routes`; bind its named outputs
  through `args_from` without replanning each phase. Use few scene-valid candidates:
  `first_valid` for known preference, `min_joint_travel` for IK travel comparison.
  Cost is not time or collision assurance. Preserve key poses with
  `path_spec_from_poses`; choose clearance and a nearby useful retreat from the
  scene, not a routine return to the grasp point. Coarser IK does not relax FK checks.

## Execute and Review

Use repository-root module commands from README: SDK via `.venv/bin/python`,
vision/serial via system `python3`. Check routes before motion. A changed
scene/state/tool/load requires reassessment; executed or outcome-unknown routes
must not be replayed, including after renaming or a new run ID.

For known-object pickup, bind every lift/carrying route's `holding_position` to
this run's `close_gripper.actual` and require local `holding_verified=true`.
The tool verifies fresh stable contact before/after and monitors during motion;
omit the binding after release. Use intermediate images when identity, grasp
geometry, slip or scene change is uncertain. Contact does not identify the object;
retain final placement verification using a new image, never the initial frame.

At `needs_review`, inspect evidence immediately and submit the guide's resume
command; a conversational verdict does not advance the run. Pass continues prepared
stages; fail/unknown stops and returns control to reassess. Only a clean pending
review can resume. On `tool_error`, stop dependent actions, inspect the error and
relevant state, and reobserve/replan unexecuted work under a new run ID/revision.
Preserve old logs/attempts; do not bypass failed guards or repair shared SDK code
inside a physical task. Report framework defects as blockers.

## Timing and Delivery

Target request-to-acceptance within 300 seconds for current pick/place, without
skipping protection/evidence, increasing speed or killing a worker. Pass the actual
request timestamp via `--requested-at`; the guide explains the overlapping timing
fields. Wait on the existing worker for 30-60 seconds when appropriate; do not
interleave empty polls, log tails and directory scans. Inspect a run log for an
error, unexpected delay or user question, not routine stage progress.

Keep execution quiet: report blockers, answer interruptions and give the final
outcome; use only minimal factual updates if the host requires them. On completed,
report immediately from the saved summary: outcome, separate robot/gripper/object
states, evidence paths, material unknowns and total elapsed time (main delay if
above target). Do not delay for README edits or task-script cleanup, or write
reports while waiting for motion/user input. Durable facts can be maintained later.

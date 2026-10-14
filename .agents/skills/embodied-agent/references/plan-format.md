# Plan Format

Use this reference for task construction and review; workflow decisions are in
[SKILL.md](../SKILL.md), device limits in the [tool contract](../../../../docs/工具与数据契约.md).
[Schema v2](../../../../config/task-plan.schema.json) requires contract_version,
adapter_id/capability_version, run_id, revision, based_on_state, goal, stages and
success. IDs are bookkeeping, not measurements. No separate task/state files or
plan-acceptance workflow are required.

## Entry Points

Run from the repository root with system Python; tools isolate SDK workers:

```bash
python3 scripts/validate_task_plan.py --adapter xmate --list-tools
python3 scripts/validate_task_plan.py path/to/plan.json --adapter xmate
python3 -m scripts.hardware.sensor_client status
# Only connect missing sensors when idle and ownership permits:
python3 -m scripts.hardware.sensor_client connect --camera --gripper
python3 scripts/run_tool.py TOOL --adapter xmate --args args.json
```

Validation checks structure, registered tools, arguments and dependencies without
hardware access; xMate also rejects repeated checked-route paths/references.
It does not check freshness, collision safety or goal coverage.
`run_tool.py` accepts stdin JSON if --args is omitted, returns
`{adapter_id, tool, result}`, and errors to stderr/nonzero without retry.
`observe` takes `{"sources":["rgb","gripper"]}`; `robot_state` takes `{}`.
Observe saves image/metadata and feedback, not localization or a task verdict.
Resident service/sensors must already be connected; no automatic direct fallback.

## Plan Assembly

Use `PlanBuilder` in the same SDK-free process as scene geometry. It fills versions
from the installed adapter and validates before writing; it invokes no tools.
The following is a construction skeleton, not a complete pick/place plan:

```python
from scripts.runtime.plan_builder import PlanBuilder, local, model

p = PlanBuilder(adapter="xmate", run_id="NEW_UNIQUE_RUN", goal=task_goal)
p.add("preflight", "plan_joint_routes", {"batch_spec": str(batch_path)},
      checks=[local("preflight_ok", "preflight.status", "checked")],
      expected="All remaining motion phases checked")
p.add("approach", "execute_joint_route",
      args_from={"checked_route_path": "preflight.routes.approach"},
      checks=[local("approach_ok", "approach.status", "completed")],
      expected="Reach the scene-derived grasp pose")
# Insert task-specific opening, closing, guarded carrying, release and retreat.
# Final observation must follow release and a checked retreat:
p.add("final", "observe", {"sources": ["rgb", "gripper"]},
      checks=[model("final_ok", acceptance_condition, ["final.rgb", "final.gripper"])],
      expected=acceptance_condition)
p.write(task_dir / "plan.json", success=["final_ok"])
```

`add` defaults requires to all preceding-stage checks; override with `requires=[...]`.
`on_fail`/`on_unknown` default to stop. Helpers are
`local(id, ref, value, op="eq")` and `model(id, condition, evidence)`.
`build(success=[...])` returns the validated dictionary without writing.
Pass revision/based_on_state at construction when needed.

Insert carrying stages after this task's `close_gripper` stage named `close`,
and before release/retreat/final:

```python
for name in ("lift", "transfer"):
    p.add(name, "execute_joint_route",
          args_from={"checked_route_path": "preflight.routes." + name,
                     "holding_position": "close.actual"},
          checks=[local(name + "_ok", name + ".status", "completed"),
                  local(name + "_held", name + ".holding_verified", True)],
          expected="Complete checked motion with stable contact")
```

Bind the fresh closing register value on **every** carrying route; omit it after
release. The tool checks five fresh samples before/after and monitors during motion.
Do not substitute an old contact value or millimetres. Exact thresholds and
failure semantics: [carrying feedback](../../../../docs/工具与数据契约.md#携物反馈).
Identity/geometry ambiguity may require an intermediate image; final placement
always needs new visual evidence.

## Batch Paths

Call `plan_joint_routes` with `{"batch_spec":"path/to/batch.json"}`. Each path_spec
is an inline object from `path_spec_from_poses`, not a filename:

```python
batch = {"selection": "first_valid", "candidates": [{
    "id": "preferred", "phases": [
        {"id": "approach", "path_spec": approach_spec},
        {"id": "lift", "path_spec": lift_spec},
        {"id": "transfer", "path_spec": transfer_spec},
        {"id": "retreat", "path_spec": retreat_spec},
    ]
}]}
```

Use 1..16 candidates with 1..32 identically named/ordered phases each; IDs use
lowercase initial letters, digits and underscores. `first_valid` (default) returns
the first fully checked candidate. `min_joint_travel` solves candidates' IK first,
ranks by summed per-segment maximum-axis travel (ties retain order), then checks
until one passes. Cost is not measured time or collision assurance; keep candidates
few and scene-valid. Geometry rejection can try another; device/ownership errors stop.

The first phase starts from live joints, later phases from predicted endpoints;
each phase ends at a fine stop. Consume `preflight.routes.PHASE` through args_from,
including across review handoffs, without duplicate per-phase planning. Execution
still verifies actual start/tool/load/frames. Single `plan_joint_route` remains
available when later geometry is genuinely unknown; it takes a path_spec file and
returns checked_route_path. The helpers below provide its inputs; custom formats
and SDK constraints are in the tool contract.

## One-Process Vision

System-Python helpers, not registered tools. Poses/ROIs/dimensions come from this
task, using the active calibration and original-image pixel coordinates.

| Import | Interface and result |
| --- | --- |
| scripts.vision.scene_batch | `analyze_frame(image_path, jobs, workers=2)`: decode once, parallel named `fn(image_bgr)` callbacks; returns image path/hash, per-job result/error and elapsed_s; no capture/model calls |
| scripts.vision.block_top | `load_plane(Path(session), top_height_m=...)`: calibrated intrinsics/transform/table; `detect_top(image, plane, size_m, roi)`: explicit red rectangular top fit, corners_px/center_base_m/checks or TopFaceNotFound |
| scripts.vision.placement_rgb | `plane_points(pixels, calibration, clearance_m)`: N×3 base-metre points on a known-height plane, not box detection/free-space proof; `rim_candidates(image, reference, calibration, rim_m, roi)`: candidates/lines using two actual frames, never the same frame twice |
| scripts.hardware.route_geometry | `path_spec_from_poses(poses, tcp_bounds_m, rate=None, translation_step_m=.02, rotation_step_deg=5., blend_radius_mm=1.)`: rate defaults to control.yaml; complete inline spec preserving all supplied keypoints; default 20 mm/5° intermediate sampling, refine for constrained geometry |
| scripts.hardware.route_geometry | `sample_tcp_path(poses, translation_step_m=..., rotation_step_deg=...)`: sampled row-major 4×4 matrices after the first pose; poses are `[x,y,z,rx,ry,rz]`, metres/radians, SDK endInRef, RPY extrinsic xyz; at least two poses; first pose is current TCP |

```python
from pathlib import Path
import cv2
from scripts.vision.block_top import load_plane, detect_top
from scripts.vision.placement_rgb import plane_points
from scripts.vision.scene_batch import analyze_frame

cal = load_plane(Path(session), top_height_m=object_height_m)
cv2.setNumThreads(1)  # Avoid nested pools on the current two-core host.
scene = analyze_frame(image_path, {
    "target": lambda image: detect_top(image, cal, object_size_m, target_roi),
    "rim_projection": lambda image: plane_points(rim_pixels, cal, rim_height_m).tolist(),
})
```

Share image/calibration read-only; copy for drawing, combine dependent calculations
afterward, save one scene result/overlay. Failed jobs remain unknown. Inspect
occupancy/tool clearance separately; computed coordinates do not prove free space.

## Stage Contract

Stages contain id, skill, args, requires, expected, checks, on_fail/on_unknown.
Requires names earlier checks; success names final-stage checks. Rules:

| Item | Meaning |
| --- | --- |
| local | `{ref, op, value}` with eq/ne/lt/le/gt/ge/in; missing/null/nonfinite/incompatible values yield unknown, no boolean-number coercion; only numbers support ordered comparisons |
| model | Describe the object effect and reference real evidence; current agent judges, no extra model service. Missing outputs cannot be invented from expected text |
| references | `stage_id.field.subfield`, case-sensitive object keys; checks/evidence may use current/earlier stages. Local evidence includes its compared field or a parent. No array indices, interpolation or expressions |
| args_from | Top-level tool argument → earlier-stage result reference; argument must exist in tool Schema and not also in args. Resolve and revalidate before invoking; missing/invalid bindings stop before the tool |
| checks | local fail/unknown stops; model produces needs_review. on_fail/on_unknown describe next_action intent, not automatic observation, revision or retry |

A new run uses a unique run_id even when revision changes. One exclusive
`logs/runs/plan-<run_id>.jsonl` stores the plan, pre-call attempts, results, verdicts
and stop reason. Review restores prior outputs without replay; tool_error or an
interrupted run cannot resume. Reobserve, revise only unexecuted work in plan.json,
increment revision and use a new run_id while preserving old logs/attempts.

## Run and Review

```bash
python3 scripts/run_plan.py path/to/plan.json --adapter xmate --requested-at ISO_TIMESTAMP_WITH_TIMEZONE
python3 scripts/run_plan.py --adapter xmate --resume RUN_LOG \
  --review-id EXACT_PENDING_ID --verdict final_ok=pass --reason 'Judgment grounded in the fresh evidence'
```

At needs_review the CLI exits. Inspect its evidence directly, then repeat --verdict
once for every requested check, using pass/fail/unknown and the exact review ID.
Only a clean pending review can continue; duplicate/stale/incomplete responses or
concurrent submissions are rejected. Pass continues the next stage, fail/unknown
stops. If the scene changed, reassess rather than approving stale evidence.

Equivalent `--resume RUN_LOG --review RESPONSE` accepts a file or `-` for stdin:

```json
{
  "run_id": "FROM_REQUEST", "revision": 1, "stage_id": "FROM_REQUEST",
  "review_id": "FROM_REQUEST", "verdicts": {"REQUESTED_CHECK_ID": "pass"},
  "reason": "Judgment grounded in the supplied evidence."
}
```

Copy **all** identities, including revision, from the request. Verdicts are recorded
under the log lock before remaining tools run. Python synchronous reviewer callbacks
are also supported. completed means declared checks passed, not proof they cover
the original user goal. Exit codes: 0 completed, 2 stopped/needs_review, 1 input/log error.

Stdout normally contains status, log_path, checks and any review; --full-output
includes results. completed writes `<run-log-stem>.summary.json` and returns
summary_path. `--requested-at` must be the actual request time, never a guessed
stopwatch start. timing has plan_elapsed_s, tool_elapsed_s, review_wait_s and, when
supplied, request_elapsed_s. They overlap: do not sum them. Without requested-at,
initial observation/planning is excluded; summary timing ends at the verdict, not
the later chat reply. Report from the summary immediately per the skill.

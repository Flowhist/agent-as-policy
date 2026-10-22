---
name: embodied-agent
description: Execute or review robot tasks using the existing guarded device tools.
---

# Embodied Agent

Start here; README, schemas and references are on demand, not prerequisite reading.
Complete physical tasks use plan.json and run_plan for sequencing/checks/stops.
Direct tools are for initial observations, read-only queries and explicit debugging.
Follow the user's current scope; generating a plan is not executing it.

## Start

From the repository root, check ownership: `python3 -m scripts.hardware.sensor_client status`.
Do not acquire devices while busy/owned; web robot must be disconnected for task SDK use.
Connect only missing sensors when idle: `python3 -m scripts.hardware.sensor_client connect --camera --gripper`
(use only missing flags, not automatic fault recovery). Service status is not live SDK state.

Invoke a tool with JSON on stdin, or `--args path/to/args.json`:
```bash
python3 scripts/run_tool.py observe --adapter xmate <<'JSON'
{"sources":["rgb","gripper"]}
JSON
```

| Tool | Arguments / role |
| --- | --- |
| observe | sources: rgb/gripper; fresh evidence from connected resident sensors |
| robot_state | {}; query idle robot in isolated SDK process |
| move_gripper | target_position: 0..1000; at most 500 change |
| close_gripper | target_position: 0..999; initialized, fully open empty start at 1000 |
| plan_joint_routes | batch_spec: path; check known motion phases together |
| plan_joint_route | path_spec: path; check one sampled TCP path |
| check_joint_route | route_path: path; check explicit joint route |
| execute_joint_route | checked_route_path: returned path; optional holding_position |

If arguments/results are unfamiliar, inspect only that tool:
`python3 scripts/validate_task_plan.py --adapter xmate --tool TOOL`.
Use actual returned paths/values, not invented results. New routes/gripper actions use
config/control.yaml defaults; explicit rate overrides require current user intent.

## Plan and Run

For short plans, run Python in the terminal without saving build.py. This only writes
and validates JSON; execute separately. Direct schema-v2 JSON is also accepted.
Minimal final-review example, not a complete manipulation plan:

```bash
python3 - <<'PY'
from uuid import uuid4
from scripts.runtime.plan_builder import PlanBuilder, local, model
p = PlanBuilder(adapter="xmate", run_id="task-" + uuid4().hex,
                goal="Verify the requested final placement")
p.add("final", "observe", {"sources": ["rgb", "gripper"]},
      expected="Fresh placement evidence",
      checks=[model("final_ok", "The intended object is at the requested destination",
                    ["final.rgb", "final.gripper"])])
print(p.write("captures/task-" + p.plan["run_id"] + "/plan.json", success=["final_ok"]))
PY
```

Replace/extend stages for the current goal. `local(id, "stage.field", value, op="eq")`
checks results; `model(id, condition, evidence)` hands judgment to this agent.
`add` defaults to requiring the previous stage's checks and stopping on fail/unknown;
use `requires=[check_id, ...]` for other dependencies and
`args_from={"argument": "earlier_stage.field"}` for live result bindings.
`success` names final-stage checks. Save generator scripts only for complex debugging/reuse.

Run the returned path: `python3 scripts/run_plan.py PLAN_PATH --adapter xmate`.
Add `--requested-at ISO_TIMESTAMP_WITH_TIMEZONE` only with the actual request time.
At `needs_review`, inspect the supplied evidence, then resume with:
`python3 scripts/run_plan.py --adapter xmate --resume RUN_LOG --review-id EXACT_PENDING_ID --verdict CHECK_ID=pass --reason 'Evidence-based judgment'`.
Repeat `--verdict` for every requested check; use fail/unknown when appropriate.
A chat judgment alone does not resume the plan. Do not interleave manual actions.

## Decide From Evidence

- Inspect target, destination and obstacles in one frame; reuse it until change or
  uncertainty requires observation. Known dimensions may be reused, not old poses.
- Confirm destination reachability and arm/finger/object/cable clearance before grasping.
  IK/FK is not collision proof; MoveAbsJ is not Cartesian-linear. Account for carried
  extents, rotate above obstacles, use suitable descent/exit and confined-space release.
- Check every route before execution. On each lift/carry pass this grasp's actual
  closing position as holding_position and require holding_verified=true; omit after
  release. Contact does not identify an object. Final placement requires a new image.
- Errors/unknown outcomes stop, not retry. Reobserve and plan only remaining work
  with a new run_id; never restart the old plan. Direct debug calls lack plan checks.
  Preserve SDK locks, process isolation, manual/idle guards and feedback. Never replay
  attempted routes, even renamed; reassess state/tool/load/scene changes. No automatic
  mode change, alarm clearing or homing. Serial close is not gripper stop; power-off
  does not release an object; software stop is not emergency stop.

For advanced fields see [plan reference](references/plan-format.md#plan-assembly).
[Geometry APIs](references/plan-format.md#one-process-vision),
[device details](../../../docs/工具与数据契约.md) and [installation facts](../../../README.md#当前设备与物件参数)
are references only when needed; this entry adds no new perception or motion capability.

Keep execution quiet, wait on existing workers, then report actual outcome, separate
robot/gripper/object states, evidence and elapsed time. Do not delay for maintenance.

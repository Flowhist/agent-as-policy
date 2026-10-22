"""Check a plan's structure and tool arguments without executing it."""

import argparse
import json
from pathlib import Path
import sys

from jsonschema.exceptions import SchemaError

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.runtime.contract_io import load_json, numeric_errors, pointer, schema_validator

ROOT = Path(__file__).resolve().parents[1]
SCHEMA = ROOT / "config/task-plan.schema.json"


def report(plan, errors):
    data = plan if isinstance(plan, dict) else {}
    return {
        "valid": not errors,
        "contract_version": (data.get("contract_version")
                             if type(data.get("contract_version")) is int else None),
        "run_id": data.get("run_id") if isinstance(data.get("run_id"), str) else None,
        "revision": data.get("revision") if type(data.get("revision")) is int else None,
        "based_on_state": (data.get("based_on_state")
                           if type(data.get("based_on_state")) is int else None),
        "stage_count": len(data["stages"]) if isinstance(data.get("stages"), list) else 0,
        "errors": errors,
    }


def validate_plan(plan, *, run_id=None, state_version=None, revision=None, registry=None):
    version = plan.get("contract_version") if isinstance(plan, dict) else None
    if type(version) is not int or version != 2:
        return report(plan, [{"path": "/contract_version", "message": "Only plan contract version 2 is supported"}])
    schema = load_json(SCHEMA)
    validator = schema_validator(schema)
    errors = list(numeric_errors(plan))[:32]
    if errors:
        return report(plan, errors)

    def reject(path, message):
        errors.append({"path": path, "message": str(message)[:800]})

    for error in validator.iter_errors(plan):
        reject(pointer(error.absolute_path), error.message)
        if len(errors) >= 32:
            break
    if errors:
        return report(plan, errors)

    for field, expected in (("run_id", run_id), ("based_on_state", state_version),
                            ("revision", revision)):
        if expected is None:
            continue
        if ((field == "run_id" and not isinstance(expected, str))
                or (field != "run_id" and (type(expected) is not int or expected < 0))):
            reject("/" + field, "Caller context has an invalid type or value")
        elif plan[field] != expected:
            reject("/" + field, f"Does not match caller context: expected {expected!r}")

    stage_ids, available_checks = set(), set()
    for index, stage in enumerate(plan["stages"]):
        base = f"/stages/{index}"
        if stage["id"] in stage_ids:
            reject(base + "/id", "Stage ID is already used")
        for key, ref in stage.get('args_from', {}).items():
            if ref.split('.')[0] not in stage_ids:
                reject(base + '/args_from/' + key, 'Argument must reference an earlier stage')
        stage_ids.add(stage["id"])
        for position, required in enumerate(stage["requires"]):
            if required not in available_checks:
                reject(base + f"/requires/{position}",
                       "Dependency must reference a check in an earlier stage")
        for position, check in enumerate(stage["checks"]):
            if check["id"] in available_checks:
                reject(base + f"/checks/{position}/id", "Check ID is already used")
            available_checks.add(check["id"])

    if registry is None:
        reject("/adapter_id", "Select an installed adapter explicitly")
    else:
        errors.extend(registry.plan_errors(plan))

    final_checks = {check["id"] for check in plan["stages"][-1]["checks"]}
    for position, check_id in enumerate(plan["success"]):
        if check_id not in final_checks:
            reject(f"/success/{position}", "Success must reference a check in the final stage")
    result = report(plan, errors)
    result["adapter_id"] = plan["adapter_id"]
    result["capability_version"] = plan["capability_version"]
    result["unavailable_tools"] = (
        registry.unavailable_tools(s["skill"] for s in plan["stages"])
        if registry is not None else sorted({s["skill"] for s in plan["stages"]}))
    return result


def nonnegative_int(text):
    value = int(text)
    if value < 0:
        raise argparse.ArgumentTypeError("Value must not be negative")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", nargs="?", help="JSON plan path, or - for stdin")
    parser.add_argument("--run-id", help="Expected task ID from the caller")
    parser.add_argument("--state-version", type=nonnegative_int,
                        help="Expected observation version from the caller")
    parser.add_argument("--revision", type=nonnegative_int,
                        help="Expected policy revision from the caller")
    parser.add_argument("--adapter", required=True, help="Installed adapter, e.g. xmate")
    discovery = parser.add_mutually_exclusive_group()
    discovery.add_argument("--list-tools", action="store_true", help="Print all capability definitions")
    discovery.add_argument("--tool", help="Print one tool's exact contract without executing it")
    args = parser.parse_args()
    if (args.list_tools or args.tool) and args.plan:
        parser.error("Tool discovery does not take a plan")
    if not (args.list_tools or args.tool) and not args.plan:
        parser.error("A plan is required")
    try:
        from scripts.adapters import load_adapter
        registry = load_adapter(args.adapter)
        if args.list_tools or args.tool:
            print(json.dumps(registry.describe(args.tool),
                             indent=2, ensure_ascii=False, allow_nan=False))
            return 0
        result = validate_plan(load_json(args.plan), run_id=args.run_id,
                               state_version=args.state_version, revision=args.revision,
                               registry=registry)
    except (OSError, ValueError, RecursionError, SchemaError) as exc:
        result = report(None, [{"path": "", "message": str(exc)[:800]}])
    print(json.dumps(result, indent=2, ensure_ascii=True, allow_nan=False))
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())

"""Build and validate ordinary plans without invoking any device tools."""

from copy import deepcopy
import json
from pathlib import Path

from scripts.adapters import load_adapter
from scripts.run_plan import validate_run


def local(check_id, ref, value, *, op='eq'):
    return {'id': check_id, 'mode': 'local',
            'condition': {'ref': ref, 'op': op, 'value': value}, 'evidence': [ref]}


def model(check_id, condition, evidence):
    return {'id': check_id, 'mode': 'model', 'condition': condition,
            'evidence': list(evidence)}


class PlanBuilder:
    def __init__(self, *, adapter, run_id, goal, revision=1, based_on_state=0):
        self.registry = load_adapter(adapter)
        self.plan = dict(contract_version=2, adapter_id=self.registry.adapter_id,
                         capability_version=self.registry.version, run_id=run_id,
                         goal=goal, revision=revision, based_on_state=based_on_state,
                         stages=[])

    def add(self, stage_id, tool, args=None, *, checks, expected,
            args_from=None, requires=None, on_fail='stop', on_unknown='stop'):
        stages = self.plan['stages']
        if requires is None:
            requires = [c['id'] for c in stages[-1]['checks']] if stages else []
        stage = dict(id=stage_id, skill=tool, args=args if args is not None else {},
                     checks=checks, expected=expected, requires=requires,
                     on_fail=on_fail, on_unknown=on_unknown)
        if args_from is not None:
            stage['args_from'] = args_from
        stages.append(deepcopy(stage))
        return self

    def build(self, *, success):
        plan = deepcopy(dict(self.plan, success=list(success)))
        validate_run(plan, self.registry)
        return plan

    def write(self, path, *, success):
        plan = self.build(success=success)
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(plan, indent=2, allow_nan=False) + '\n')
        return path

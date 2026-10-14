"""Run stages sequentially, stopping on failed conditions or an agent review."""

import argparse
import fcntl
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time
import uuid

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.adapters import load_adapter
from scripts.runtime.condition_checks import evaluate_condition
from scripts.runtime.contract_io import load_json
from scripts.validate_task_plan import validate_plan

ROOT = Path(__file__).resolve().parents[1]
MISSING = object()


def resolve(results, ref):
    value = results
    for key in ref.split('.'):
        if not isinstance(value, dict) or key not in value:
            return MISSING
        value = value[key]
    return value


def validate_run(plan, registry):
    validation = validate_plan(plan, registry=registry)
    if not validation['valid'] or validation['unavailable_tools']:
        raise ValueError(f'Plan cannot run: {validation}')
    available = set()
    for stage in plan['stages']:
        available.add(stage['id'])
        for check in stage['checks']:
            refs = list(check['evidence'])
            if check['mode'] == 'local':
                refs.append(check['condition']['ref'])
                if not any(check['condition']['ref'] == ref or
                           check['condition']['ref'].startswith(ref + '.')
                           for ref in check['evidence']):
                    raise ValueError('Local evidence must include the compared field or its parent')
            if any(ref.split('.')[0] not in available for ref in refs):
                raise ValueError('References must start with this or an earlier stage ID')


def run_plan(plan, registry, *, reviewer=None, log_directory=None, requested_at=None):
    """reviewer(request) optionally returns check_id -> pass/fail/unknown in-process."""
    validate_run(plan, registry)
    if requested_at is not None:
        requested_at = parse_request_time(requested_at).isoformat()
    folder = Path(log_directory) if log_directory is not None else ROOT / 'logs/runs'
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f'plan-{plan["run_id"]}.jsonl'
    # A new run is exclusive. Review continuation appends to this same log.
    with path.open('x') as log:
        fcntl.flock(log, fcntl.LOCK_EX | fcntl.LOCK_NB)
        write_event(log, 'plan', plan=plan, requested_at=requested_at)
        return advance(plan, registry, log, path, {}, {}, reviewer=reviewer)


def write_event(log, event, **data):
    log.write(json.dumps(dict(time=datetime.now(timezone.utc).isoformat(),
                              event=event, **data), allow_nan=False) + '\n')
    log.flush()
    os.fsync(log.fileno())


def valid_verdicts(response, checks):
    return (isinstance(response, dict) and set(response) == {c['check_id'] for c in checks}
            and all(v in ('pass', 'fail', 'unknown') for v in response.values()))


def parse_request_time(value):
    stamp = datetime.fromisoformat(value.replace('Z', '+00:00'))
    if stamp.tzinfo is None or stamp > datetime.now(timezone.utc):
        raise ValueError('Request time must include a timezone and not be in the future')
    return stamp


def run_timing(path):
    with Path(path).open() as stream:
        events = [json.loads(line) for line in stream]
    first, last = events[0], events[-1]
    end = datetime.fromisoformat(last['time'])
    timing = {'plan_elapsed_s': (end-datetime.fromisoformat(first['time'])).total_seconds(),
              'tool_elapsed_s': sum(e['elapsed_s'] for e in events if e['event'] == 'result'),
              'review_wait_s': 0.}
    pending = None
    for event in events:
        if event['event'] == 'review':
            pending = datetime.fromisoformat(event['time'])
        elif event['event'] == 'review_result' and pending is not None:
            timing['review_wait_s'] += (datetime.fromisoformat(event['time'])-pending).total_seconds()
            pending = None
    if first.get('requested_at'):
        timing['request_elapsed_s'] = (end-datetime.fromisoformat(first['requested_at'])).total_seconds()
    return {key: round(value, 3) for key, value in timing.items()}


def resume_review(path, response, registry):
    """Continue only a clean review handoff, never an interrupted tool execution."""
    path = Path(path)
    with path.open('r+') as log:
        fcntl.flock(log, fcntl.LOCK_EX | fcntl.LOCK_NB)
        events = [json.loads(line) for line in log]
        if (not events or events[0].get('event') != 'plan'
                or events[-1].get('event') != 'finished'
                or events[-1].get('status') != 'needs_review'):
            raise ValueError('Run is not waiting for review; completed/interrupted actions cannot be resumed')
        plan, last = events[0]['plan'], events[-1]
        validate_run(plan, registry)
        request = last['review']
        identity = ('run_id', 'revision', 'stage_id', 'review_id')
        # The unique review ID still binds a compact CLI response to this exact handoff.
        if isinstance(response, dict) and set(response) == {'review_id', 'verdicts', 'reason'}:
            response = {**{key: request[key] for key in identity}, **response}
        if (not isinstance(response, dict) or set(response) != {*identity, 'verdicts', 'reason'}
                or any(key not in request or type(response[key]) is not type(request[key])
                       or response[key] != request[key] for key in identity)
                or not isinstance(response['reason'], str) or not response['reason'].strip()
                or not valid_verdicts(response['verdicts'], request['checks'])):
            raise ValueError('Review must match the pending request and contain every verdict plus a reason')
        results = {e['stage_id']: e['result'] for e in events if e['event'] == 'result'}
        verdicts = dict(last['checks'])
        index = next(i for i, stage in enumerate(plan['stages']) if stage['id'] == request['stage_id'])
        log.seek(0, os.SEEK_END)
        # Consume the response durably before any remaining stage can execute.
        write_event(log, 'review_result', stage_id=request['stage_id'], review_id=request['review_id'],
             verdicts=response['verdicts'], reason=response['reason'])
        verdicts.update(response['verdicts'])
        return advance(plan, registry, log, path, results, verdicts, start=index+1,
                       reviewed_stage=plan['stages'][index])


def advance(plan, registry, log, path, results, verdicts, *, start=0,
            reviewed_stage=None, reviewer=None):
    # Keep the stage loop shared by a new run and a review continuation.
    def emit(event, **data):
        write_event(log, event, **data)

    def finish(status, **details):
        outcome = dict(run_id=plan['run_id'], revision=plan['revision'], status=status,
                       log_path=str(path), results=results, checks=verdicts, **details)
        emit('finished', status=status, checks=verdicts, **details)
        outcome['timing'] = run_timing(path)
        return outcome

    if reviewed_stage is not None:
        for status in ('fail', 'unknown'):
            if status in (verdicts.get(c['id']) for c in reviewed_stage['checks']):
                return finish(status, stage_id=reviewed_stage['id'],
                              next_action=reviewed_stage['on_' + status])
    for stage in plan['stages'][start:]:
        sid = stage['id']
        if any(verdicts.get(ref) != 'pass' for ref in stage['requires']):
            return finish('blocked', stage_id=sid, reason='dependency_not_passed')
        try:
            args = dict(stage['args'])
            for key, ref in stage.get('args_from', {}).items():
                value = resolve(results, ref)
                if value is MISSING:
                    raise ValueError(f'Missing argument evidence: {ref}')
                args[key] = value
            emit('started', stage_id=sid, tool=stage['skill'], args=args)
            started = time.monotonic()
            results[sid] = registry.invoke(stage['skill'], args)
        except (Exception, KeyboardInterrupt) as exc:
            return finish('tool_error', stage_id=sid, outcome='unknown', error=str(exc))
        emit('result', stage_id=sid, result=results[sid], elapsed_s=time.monotonic()-started)
        pending = []
        for check in stage['checks']:
            evidence = {ref: resolve(results, ref) for ref in check['evidence']}
            if any(value is MISSING for value in evidence.values()):
                verdict = 'unknown'
            elif check['mode'] == 'local':
                ref = check['condition']['ref']
                value = resolve(results, ref)
                verdict = evaluate_condition(check['condition'],
                                             {} if value is MISSING else {ref: value})
            else:
                pending.append(dict(check_id=check['id'], condition=check['condition'],
                                    evidence=evidence))
                continue
            verdicts[check['id']] = verdict
            emit('check', stage_id=sid, check_id=check['id'], verdict=verdict)
        known = [verdicts[c['id']] for c in stage['checks'] if c['id'] in verdicts]
        if 'fail' in known or 'unknown' in known:
            status = 'fail' if 'fail' in known else 'unknown'
            return finish(status, stage_id=sid, next_action=stage['on_' + status])
        if pending:
            request = dict(run_id=plan['run_id'], revision=plan['revision'],
                           goal=plan['goal'], stage_id=sid, expected=stage['expected'],
                           checks=pending, review_id=uuid.uuid4().hex)
            emit('review', request=request)
            if reviewer is None:
                return finish('needs_review', stage_id=sid, review=request)
            try:
                response = reviewer(request)
                if not valid_verdicts(response, pending):
                    raise ValueError('Reviewer must return one verdict per requested check')
            except (Exception, KeyboardInterrupt) as exc:
                return finish('review_error', stage_id=sid, error=str(exc))
            verdicts.update(response)
            emit('review_result', stage_id=sid, verdicts=response)
            for status in ('fail', 'unknown'):
                if status in response.values():
                    return finish(status, stage_id=sid, next_action=stage['on_' + status])
    return finish('completed' if all(verdicts.get(c) == 'pass' for c in plan['success'])
                  else 'unknown')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan', nargs='?', help='JSON plan file or - for stdin')
    parser.add_argument('--adapter', required=True)
    parser.add_argument('--resume', type=Path, help='Existing run JSONL waiting for review')
    parser.add_argument('--review', help='Agent response JSON file, or - for stdin')
    parser.add_argument('--review-id', help='Pending review ID for a compact response')
    parser.add_argument('--verdict', action='append', default=[], metavar='CHECK=VERDICT')
    parser.add_argument('--reason', help='Evidence-based reason for the compact response')
    parser.add_argument('--requested-at', help='Request timestamp with timezone; new runs only')
    parser.add_argument('--full-output', action='store_true', help='Include all accumulated tool results on stdout')
    args = parser.parse_args()
    compact = args.review_id is not None or bool(args.verdict) or args.reason is not None
    if (args.resume is None and (args.plan is None or args.review is not None or compact)
            or args.resume is not None and (args.plan is not None or args.requested_at is not None
                or (args.review is None and not compact) or (args.review is not None and compact))
            or compact and not (args.review_id and args.verdict and args.reason)):
        parser.error('Provide PLAN, or --resume with --review RESPONSE or --review-id/--verdict/--reason')
    try:
        registry = load_adapter(args.adapter)
        if compact:
            verdicts = {}
            for item in args.verdict:
                key, sep, value = item.partition('=')
                if not sep or key in verdicts or value not in ('pass', 'fail', 'unknown'):
                    raise ValueError('Each --verdict must be a unique CHECK=pass/fail/unknown')
                verdicts[key] = value
            response = dict(review_id=args.review_id, verdicts=verdicts, reason=args.reason)
        else:
            response = load_json(args.review) if args.resume else None
        result = (resume_review(args.resume, response, registry) if args.resume
                  else run_plan(load_json(args.plan), registry, requested_at=args.requested_at))
        if result['status'] == 'completed':
            summary = Path(result['log_path']).with_suffix('.summary.json')
            try:
                summary.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
                result['summary_path'] = str(summary)
            except OSError as exc:
                result['summary_error'] = str(exc)
        display = result if args.full_output else {k: v for k, v in result.items() if k != 'results'}
        print(json.dumps(display, indent=2, allow_nan=False))
        return 0 if result['status'] == 'completed' else 2
    except Exception as exc:
        print(json.dumps({'error': str(exc)}), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())

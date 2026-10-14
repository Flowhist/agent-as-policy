"""Check or execute a bounded joint route in an exclusive SDK process."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import sys
import time
import uuid

from scripts.hardware.sdk_runtime import sdk_runtime
from scripts.hardware.control_settings import load_settings, validate_rate
from scripts.hardware.sensor_client import SensorClient
from scripts.hardware.web_hardware import sdk_call as call
from scripts.hardware.route_geometry import compact_joints
from scripts.hardware.grasp_orientation_ik import check_sampled_paths

ROOT = Path(__file__).resolve().parents[2]


class RouteRejected(RuntimeError):
    """A candidate path failed geometry checks; no hardware action was sent."""


def vector(value, length):
    if (not isinstance(value, list) or len(value) != length
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
        raise ValueError(f'Expected {length} finite numbers')
    return value


def validate_route(route):
    required = {'start_joints_deg', 'waypoints_deg', 'tcp_bounds_m', 'rate'}
    if (not isinstance(route, dict) or not required <= set(route)
            or set(route) - required - {'blend_radius_mm'}):
        raise ValueError('Route needs start_joints_deg, waypoints_deg, tcp_bounds_m, rate; optional blend_radius_mm')
    blend_radius(route)
    vector(route['start_joints_deg'], 6)
    points = route['waypoints_deg']
    if not isinstance(points, list) or not 1 <= len(points) <= 100:
        raise ValueError('Provide 1..100 joint waypoints')
    for point in points:
        vector(point, 6)
    bounds = route['tcp_bounds_m']
    if not isinstance(bounds, list) or len(bounds) != 3:
        raise ValueError('tcp_bounds_m needs [min,max] for X,Y,Z in endInRef metres')
    for pair in bounds:
        vector(pair, 2)
        if pair[0] >= pair[1]:
            raise ValueError('TCP bounds must have positive width')
    rate = route['rate']
    validate_rate(rate)


def blend_radius(route):
    value = route.get('blend_radius_mm', 0.)
    # SDK values below 1 mm mean fine positioning, not a smaller blend.
    if (type(value) not in (float, int) or not math.isfinite(value)
            or not (value == 0 or 1 <= value <= 5)):
        raise ValueError('blend_radius_mm must be 0 (fine) or 1..5 mm')
    return value


def digest(route):
    return hashlib.sha256(json.dumps(route, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def write_json(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())


def context(robot):
    tool = call(robot.toolset)
    # Tool, load and reference changes invalidate a previously checked route.
    return tool, {
        'end': list(tool.end.trans) + list(tool.end.rpy),
        'ref': list(tool.ref.trans) + list(tool.ref.rpy),
        'base': list(call(robot.baseFrame)),
        'load': [tool.load.mass] + list(tool.load.cog) + list(tool.load.inertia),
    }


def in_bounds(pose, bounds):
    return all(lo <= value <= hi for value, (lo, hi) in zip(pose[:3], bounds))


def checked(robot, sdk, route, check_control, cached=None, *, predicted_start=False, fk_cache=None,
            metrics=None):
    started = time.monotonic()
    metrics = {} if metrics is None else metrics
    metrics.update(sample_rows=0, fk_calls=0, fk_cache_hits=0, fk_elapsed_s=0.)
    check_control()
    info = call(robot.robotInfo)
    if info.type != 'XMS4-R800-B4S1A2C' or info.joint_num != 6:
        raise RuntimeError('Unexpected robot model')
    if (call(robot.operateMode) != sdk.OperateMode.manual
            or call(robot.operationState) != sdk.OperationState.idle):
        raise RuntimeError('Expected manual and idle')
    actual = vector(list(map(math.degrees, call(robot.jointPos)[:6])), 6)
    if not predicted_start and max(abs(a-b) for a, b in zip(actual, route['start_joints_deg'])) > .03:
        raise RuntimeError('Robot differs from route start; reobserve and replan')
    tool, snapshot = context(robot)
    for values in snapshot.values():
        vector(values, len(values))
    bounds = sdk.PyTypeVectorArrayDouble2()
    call(robot.getSoftLimit, bounds)
    limits = bounds.content()[:6]
    if len(limits) != 6:
        raise RuntimeError('Missing joint limits')
    limits = [vector(list(pair), 2) for pair in limits]
    if any(lo >= hi for lo, hi in limits):
        raise RuntimeError('Invalid joint limits')
    snapshot['limits_rad'] = [v for pair in limits for v in pair]
    if cached is not None:
        old = cached['context']
        if (old.keys() != snapshot.keys()
                or any(len(old[k]) != len(v) or any(abs(a-b) > 1e-6
                       for a,b in zip(vector(old[k], len(v)), v))
                       for k,v in snapshot.items())):
            raise RuntimeError('Tool, load, frame or limits changed; replan')
    model = robot.model()
    fk_cache = {} if fk_cache is None else fk_cache
    context_key = digest(snapshot)
    segments = []
    previous = route['start_joints_deg']
    for target in ([] if cached is not None else route['waypoints_deg']):
        count = max(1, math.ceil(max(abs(a-b) for a, b in zip(previous, target)) / 1.0))
        rows = []
        for index in range(count+1):
            metrics['sample_rows'] += 1
            check_control()
            q = [a+(b-a)*index/count for a, b in zip(previous, target)]
            qr = list(map(math.radians, q))
            if any(not lo+math.radians(.5) <= v <= hi-math.radians(.5)
                   for v, (lo, hi) in zip(qr, limits)):
                raise RouteRejected('Joint limit margin below 0.5 degrees')
            if abs(q[4]) < 5:
                raise RouteRejected('Wrist J5 is within 5 degrees of zero')
            key = (context_key, *q)
            if key not in fk_cache:
                fk_started = time.monotonic()
                fk = call(model.calcFk, qr, tool)
                metrics['fk_elapsed_s'] += time.monotonic() - fk_started
                metrics['fk_calls'] += 1
                fk_cache[key] = vector(list(fk.trans)+list(fk.rpy), 6)
            else:
                metrics['fk_cache_hits'] += 1
            pose = fk_cache[key]
            if not in_bounds(pose, route['tcp_bounds_m']):
                raise RouteRejected('Sampled TCP outside task bounds')
            if blend_radius(route) and len(route['waypoints_deg']) > 1:
                margin = blend_radius(route) / 1000
                if not all(lo+margin <= v <= hi-margin
                           for v, (lo, hi) in zip(pose[:3], route['tcp_bounds_m'])):
                    raise RouteRejected('Insufficient TCP bounds margin for blending; replan or use blend_radius_mm=0')
            rows.append({'joints_deg': q, 'tcp_m_rad': pose})
        segments.append(rows)
        previous = target
    measured = vector(list(call(robot.posture, sdk.CoordinateType.endInRef)), 6)
    actual_fk = call(model.calcFk, list(map(math.radians, actual)), tool)
    if ((not predicted_start and not in_bounds(measured, route['tcp_bounds_m']))
            or max(abs(a-b) for a, b in zip(measured[:3], actual_fk.trans)) > .0002):
        raise RuntimeError('Measured TCP outside bounds or inconsistent with FK')
    metrics['elapsed_s'] = time.monotonic() - started
    return snapshot, cached['segments'] if cached is not None else segments


def checked_digest(document):
    return digest({key: document[key] for key in ('route', 'context', 'segments')})


def verify_check_record(document):
    path = Path(document['log_path']).resolve()
    if path.parent != (ROOT / 'logs/runs').resolve():
        raise ValueError('Checked route must reference its local check log')
    expected = checked_digest(document)
    with path.open() as stream:
        if not any(json.loads(line).get('check_digest') == expected for line in stream):
            raise ValueError('Checked samples lack matching check record; check a fresh unused route')


def plan_route(robot, sdk, specification, check_control, emit, *, start_joints_deg=None):
    if isinstance(specification, dict) and 'rate' not in specification:
        specification = {**specification, 'rate': load_settings()['robot']['sdk_rate']}
    required = {'tcp_matrices_row_major', 'tcp_bounds_m', 'rate'}
    if (not isinstance(specification, dict) or not required <= set(specification)
            or set(specification) - required - {'blend_radius_mm', 'keypoint_indices'}):
        raise ValueError('Path needs tcp_matrices_row_major, tcp_bounds_m and rate')
    validate_rate(specification['rate'])
    radius = blend_radius({'blend_radius_mm': specification.get('blend_radius_mm', 1.)})
    matrices = specification['tcp_matrices_row_major']
    if not isinstance(matrices, list) or not 1 <= len(matrices) <= 10000:
        raise ValueError('Provide 1..10000 sampled target matrices')
    keys = specification.get('keypoint_indices', [len(matrices)-1])
    if (not isinstance(keys, list) or not keys
            or any(type(i) is not int or not 0 <= i < len(matrices) for i in keys)
            or keys != sorted(set(keys)) or keys[-1] != len(matrices)-1):
        raise ValueError('keypoint_indices must be increasing and include the final sample')
    pose = vector(list(call(robot.posture, sdk.CoordinateType.endInRef)), 6)
    result = check_sampled_paths(robot, sdk, {
        'start_tcp_m_rad': pose,
        'paths': [{'name': 'task', 'tcp_matrices_row_major': matrices}]},
        check_control=check_control, start_joints_deg=start_joints_deg)
    emit('planning_result', result=result)
    path = result['paths'][0]
    if not path['complete']:
        raise RouteRejected(f'Cartesian path rejected: {path["failure"]}')
    waypoints, first, start = [], 0, result['start_joints_deg']
    for last in keys:
        samples = [s['joints_deg'] for s in path['samples'][first:last+1]]
        waypoints.extend(compact_joints(start, samples))
        start, first = samples[-1], last+1
    route = {'start_joints_deg': result['start_joints_deg'],
             'waypoints_deg': waypoints,
             'tcp_bounds_m': specification['tcp_bounds_m'], 'rate': specification['rate'],
             'blend_radius_mm': radius}
    validate_route(route)
    return route


def plan_batch(robot, sdk, document, check_control, emit, output, log_path):
    if (not isinstance(document, dict) or 'candidates' not in document
            or set(document) - {'candidates', 'selection'}):
        raise ValueError('Batch specification needs candidates and optional selection')
    selection = document.get('selection', 'first_valid')
    if selection not in ('first_valid', 'min_joint_travel'):
        raise ValueError('selection must be first_valid or min_joint_travel')
    candidates = document['candidates']
    if not isinstance(candidates, list) or not 1 <= len(candidates) <= 16:
        raise ValueError('Provide 1..16 ordered candidates')
    ids, names = set(), None
    for candidate in candidates:
        if not isinstance(candidate, dict) or set(candidate) != {'id', 'phases'}:
            raise ValueError('Candidate needs id and phases')
        cid, phases = candidate['id'], candidate['phases']
        if not isinstance(cid, str) or not re.fullmatch(r'[a-z][a-z0-9_]*', cid) or cid in ids:
            raise ValueError('Candidate IDs must be unique lowercase identifiers')
        ids.add(cid)
        if not isinstance(phases, list) or not 1 <= len(phases) <= 32:
            raise ValueError('Provide 1..32 ordered named phases')
        phase_names = []
        for entry in phases:
            if (not isinstance(entry, dict) or set(entry) != {'id', 'path_spec'}
                    or not isinstance(entry['id'], str)
                    or not re.fullmatch(r'[a-z][a-z0-9_]*', entry['id'])):
                raise ValueError('Each phase needs a lowercase id and inline path_spec')
            phase_names.append(entry['id'])
        if len(set(phase_names)) != len(phase_names):
            raise ValueError('Phase IDs must be unique')
        if names is not None and phase_names != names:
            raise ValueError('Candidates must provide the same ordered phase names')
        names = phase_names
    initial = vector(list(map(math.degrees, call(robot.jointPos)[:6])), 6)
    failures, fk_cache = [], {}

    def solved_candidates():
        for candidate in candidates:
            cid, routes, start = candidate['id'], {}, initial
            started = time.monotonic()
            try:
                for entry in candidate['phases']:
                    phase, spec = entry['id'], entry['path_spec']
                    check_control()
                    def candidate_emit(event, **data):
                        emit(event, candidate=cid, phase=phase, **data)
                    route = plan_route(robot, sdk, spec, check_control, candidate_emit,
                                       start_joints_deg=start)
                    routes[phase] = route
                    start = route['waypoints_deg'][-1]
            except RouteRejected as exc:
                check_control()
                failures.append({'candidate': cid, 'phase': phase, 'error': str(exc)})
                emit('candidate_rejected', **failures[-1])
                continue
            # This is a joint-distance proxy, not a controller time prediction.
            travel = 0.
            for route in routes.values():
                points = [route['start_joints_deg'], *route['waypoints_deg']]
                travel += sum(max(abs(a-b) for a, b in zip(p, q))
                              for p, q in zip(points, points[1:]))
            emit('candidate_cost', candidate=cid, joint_travel_deg=travel,
                 ik_elapsed_s=time.monotonic()-started)
            yield travel, cid, routes

    solved = solved_candidates()
    if selection == 'min_joint_travel':
        solved = sorted(solved, key=lambda item: item[0])
    for travel, cid, routes in solved:
        try:
            documents = {}
            for index, (phase, route) in enumerate(routes.items()):
                if (ROOT / 'logs/runs' / ('joint-route-'+digest(route)+'.attempt.json')).exists():
                    raise RuntimeError('Batch contains an attempted route; do not replay')
                metrics = {}
                snapshot, segments = checked(robot, sdk, route, check_control,
                                              predicted_start=index > 0, fk_cache=fk_cache,
                                              metrics=metrics)
                emit('phase_checked', candidate=cid, phase=phase, **metrics)
                if documents and snapshot != next(iter(documents.values()))['context']:
                    raise RuntimeError('Device context changed during batch planning')
                documents[phase] = {'status': 'checked', 'route': route, 'context': snapshot,
                                    'segments': segments, 'route_digest': digest(route),
                                    'motion_sent': False, 'log_path': str(log_path)}
            first = next(iter(documents.values()))
            checked(robot, sdk, first['route'], check_control, cached=first)
        except RouteRejected as exc:
            check_control()
            failures.append({'candidate': cid, 'phase': phase, 'error': str(exc)})
            emit('candidate_rejected', **failures[-1])
            continue
        paths = {}
        for phase, record in documents.items():
            path = output.with_name(output.stem+'-'+phase+'.json')
            emit('check_record', candidate=cid, phase=phase, check_digest=checked_digest(record))
            write_json(path, record)
            paths[phase] = str(path)
        emit('candidate_selected', candidate=cid, routes=paths,
             selection=selection, joint_travel_deg=travel)
        return {'status': 'checked', 'selected_candidate': cid, 'routes': paths,
                'rejected_candidates': failures, 'motion_sent': False, 'log_path': str(log_path)}
    raise RouteRejected('No candidate passed: '+json.dumps(failures))


def execute(robot, sdk, route, segments, limits, check_control, emit):
    def wait_idle():
        deadline = time.monotonic()+5
        while call(robot.operationState) != sdk.OperationState.idle:
            if time.monotonic() > deadline:
                raise RuntimeError('Stop did not reach idle; inspect controller')
            time.sleep(.05)

    armed = False
    try:
        check_control()
        # Require the SDK descriptor: an instance-only Python attribute would
        # appear to accept a rate without changing the native motion command.
        descriptor = getattr(sdk.MoveAbsJCommand, 'jointSpeed', None)
        if not hasattr(descriptor, '__set__'):
            raise RuntimeError('Installed SDK lacks native MoveAbsJCommand.jointSpeed')
        commands = []
        zones = [blend_radius(route)] * (len(route['waypoints_deg'])-1) + [0.]
        for target, zone in zip(route['waypoints_deg'], zones):
            command = sdk.MoveAbsJCommand(list(map(math.radians, target)), -1, zone)
            if not math.isfinite(command.zone) or not math.isclose(command.zone, zone, abs_tol=1e-9):
                raise RuntimeError('SDK blending radius readback differs from requested radius')
            command.jointSpeed = route['rate']
            actual_rate = command.jointSpeed
            if (not math.isfinite(actual_rate)
                    or not math.isclose(actual_rate, route['rate'], rel_tol=0, abs_tol=1e-9)):
                raise RuntimeError('SDK joint speed readback differs from requested rate')
            commands.append(command)
        check_control()
        call(robot.setMotionControlMode, sdk.MotionControlMode.NrtCommandMode)
        call(robot.moveReset)
        armed = True
        # The controller retains this independent multiplier across SDK sessions.
        call(robot.adjustSpeedOnline, 1.0)
        emit('speed_configured', joint_rate=route['rate'], speed_mm_s=-1,
             online_scale=1.0, waypoint_zones_mm=zones,
             sdk_module=getattr(sdk, '__file__', None))
        if blend_radius(route):
            call(robot.setAutoIgnoreZone, False)
        if call(robot.powerState) != sdk.PowerState.on:
            call(robot.setPowerState, True)
        if call(robot.powerState) != sdk.PowerState.on:
            raise RuntimeError('Power on unconfirmed')
        check_control()
        q = vector(list(map(math.degrees, call(robot.jointPos)[:6])), 6)
        if max(abs(a-b) for a,b in zip(q, route['start_joints_deg'])) > .03:
            raise RuntimeError('Route start changed before submission')
        command_id = sdk.PyString()
        call(robot.moveAppend, commands, command_id)
        emit('queue_start', command_id=command_id.content(), segment_count=len(commands))
        call(robot.moveStart)
        keys = sdk.EventInfoKey.MoveExecution
        index, deadline = 0, time.monotonic()+90
        while True:
            check_control()
            tick = time.monotonic()
            info = call(robot.queryEventInfo, sdk.Event.moveExecution)
            matched = info.get(keys.ID) == command_id.content()
            if matched:
                current = info[keys.WaypointIndex]
                if type(current) is not int or not index <= current < len(segments):
                    raise RuntimeError('Invalid or regressing controller waypoint index')
                error = info[keys.Error]
                if isinstance(error, sdk.PyErrorCode):
                    error_code = error.value()
                elif type(error) is int:
                    error_code = error
                elif isinstance(error, dict) and type(error.get('ec')) is int:
                    error_code = error['ec']
                else:
                    raise RuntimeError('Unsupported controller event error representation')
                if error_code != 0:
                    raise RuntimeError(f'Controller motion error: {error_code}')
                if current != index:
                    index, deadline = current, time.monotonic()+90
                    emit('segment_progress', segment=index)
            q = vector(list(map(math.degrees, call(robot.jointPos)[:6])), 6)
            pose = vector(list(call(robot.posture, sdk.CoordinateType.endInRef)), 6)
            # Event and joint reads are not simultaneous; a blended corner can
            # span the incoming/outgoing segment when the waypoint index changes.
            nearby = segments[max(0, index-1) if blend_radius(route) else index:
                              min(index+2, len(segments))]
            def on_segment(rows):
                start, target = rows[0]['joints_deg'], rows[-1]['joints_deg']
                margin = .5 if blend_radius(route) else .1
                # Feedback resolution stays independent of the coarser FK grid.
                count = max(1, math.ceil(max(abs(a-b) for a,b in zip(start,target)) / .1))
                joints = ([a+(b-a)*i/count for a,b in zip(start,target)]
                          for i in range(count+1))
                return (all(min(a,b)-margin <= v <= max(a,b)+margin for v,a,b in zip(q,start,target))
                        and min(max(abs(a-b) for a,b in zip(q,point)) for point in joints) <= .5)
            if (not any(on_segment(rows) for rows in nearby)
                    or any(not lo <= math.radians(v) <= hi for v,(lo,hi) in zip(q,limits))
                    or not in_bounds(pose, route['tcp_bounds_m'])):
                raise RuntimeError('Joint route or TCP bounds violated')
            if (call(robot.operateMode) != sdk.OperateMode.manual
                    or call(robot.powerState) != sdk.PowerState.on):
                raise RuntimeError('Mode or power changed')
            emit('feedback', segment=index, joints_deg=q, tcp_m_rad=pose,
                 poll_elapsed_s=time.monotonic()-tick)
            if (matched and index == len(segments)-1 and info.get(keys.ReachTarget) is True
                    and max(abs(a-b) for a,b in zip(q,route['waypoints_deg'][-1])) < .01
                    and call(robot.operationState) == sdk.OperationState.idle):
                break
            if time.monotonic() > deadline:
                raise RuntimeError('Motion timeout; do not retry')
            time.sleep(.05)
        samples = []
        for _ in range(5):
            check_control()
            time.sleep(.1)
            q = vector(list(map(math.degrees, call(robot.jointPos)[:6])), 6)
            if max(abs(a-b) for a,b in zip(q,route['waypoints_deg'][-1])) > .02:
                raise RuntimeError('Final joint error')
            samples.append(q)
        if max(max(axis)-min(axis) for axis in zip(*samples)) > .005:
            raise RuntimeError('Final joints unstable')
        emit('route_complete', joints_deg=q)
        return {'status': 'completed', 'joints_deg': q,
                'tcp_m_rad': vector(list(call(robot.posture, sdk.CoordinateType.endInRef)), 6)}
    finally:
        if armed:
            emit('stop_requested')
            try:
                call(robot.stop)
                wait_idle()
                call(robot.setPowerState, False)
                call(robot.moveReset)
                if blend_radius(route):
                    call(robot.setAutoIgnoreZone, True)
                if call(robot.powerState) != sdk.PowerState.off:
                    raise RuntimeError('Power off unconfirmed')
                emit('powered_off')
            except BaseException as exc:
                emit('cleanup_unconfirmed', message=str(exc))
                raise


def main():
    termination_requested = False
    parent = int(os.environ.get('EMBODIED_WORKER_PARENT', os.getppid()))

    def request_termination(*_):
        nonlocal termination_requested
        termination_requested = True

    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, request_termination)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('plan', 'batch', 'check', 'execute'))
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--web-url', default='http://127.0.0.1:8080')
    parser.add_argument('--holding-position', type=int,
                        help='Require contact at the measured grasp register position throughout execution')
    args = parser.parse_args()
    if args.holding_position is not None and (args.operation != 'execute'
                                              or not 0 <= args.holding_position <= 1000):
        parser.error('--holding-position requires execute and a register value in 0..1000')
    source, output = args.source.resolve(), args.output.resolve()
    if output.exists() or not output.parent.is_dir():
        raise ValueError('Output must be a new file in an existing directory')
    document = json.loads(source.read_text())
    route = document if args.operation == 'check' else document.get('route')
    route_digest = None
    if args.operation not in ('plan', 'batch'):
        validate_route(route)
        route_digest = digest(route)
    if args.operation == 'execute' and (document.get('status') != 'checked'
                                         or document.get('route_digest') != route_digest):
        raise ValueError('Not an intact checked route')
    if args.operation == 'execute':
        verify_check_record(document)
    run_dir = ROOT / 'logs/runs'
    run_dir.mkdir(parents=True, exist_ok=True)
    attempt = run_dir / ('joint-route-'+str(route_digest)+'.attempt.json')
    if route_digest is not None and attempt.exists():
        raise RuntimeError('This route was already attempted; do not replay or recheck it')
    log_path = run_dir / ('joint-route-'+uuid.uuid4().hex+'.jsonl')
    with log_path.open('x') as log:
        def emit(event, **data):
            row = {'time': datetime.now(timezone.utc).isoformat(), 'event': event, **data}
            log.write(json.dumps(row, allow_nan=False)+'\n')
            log.flush()
        emit('worker_started', pid=os.getpid(), parent_pid=parent, operation=args.operation)
        try:
            with SensorClient(args.web_url).control() as client, sdk_runtime():
                def check_control():
                    if termination_requested or os.getppid() != parent:
                        raise InterruptedError('Termination requested or parent lost; stop without replay')
                    client.check_control()

                check_control()
                import xCoreSDK_python as sdk
                os.chdir(ROOT / 'logs/sdk')
                robot = sdk.xMateRobot()
                robot.connectToRobot('192.168.0.160', '')
                try:
                    started = time.monotonic()
                    if args.operation == 'batch':
                        emit('planning_input', source=str(source), specification=document)
                        result = plan_batch(robot, sdk, document, check_control, emit, output, log_path)
                        emit('batch_complete', elapsed_s=time.monotonic()-started)
                        write_json(output, result)
                        return
                    if args.operation == 'plan':
                        emit('planning_input', source=str(source), specification=document)
                        route = plan_route(robot, sdk, document, check_control, emit)
                        route_digest = digest(route)
                        attempt = run_dir / ('joint-route-'+route_digest+'.attempt.json')
                        if attempt.exists():
                            raise RuntimeError('Planned route was already attempted; do not replay')
                        emit('planned', elapsed_s=time.monotonic()-started)
                    snapshot, segments = checked(robot, sdk, route, check_control,
                                                 cached=document if args.operation == 'execute' else None)
                    emit('checked', route=route, context=snapshot,
                         elapsed_s=time.monotonic()-started,
                         reused_samples=args.operation == 'execute')
                    if args.operation != 'execute':
                        result = {'status': 'checked', 'route': route, 'context': snapshot,
                                  'segments': segments, 'route_digest': route_digest,
                                  'motion_sent': False}
                        emit('check_record', check_digest=checked_digest(result))
                    else:
                        write_json(attempt, {'source': str(source), 'route_digest': route_digest,
                                            'log_path': str(log_path), 'route': route})
                        emit('execution_attempt', marker=str(attempt))
                        limits = list(zip(snapshot['limits_rad'][::2], snapshot['limits_rad'][1::2]))
                        gripper = client.state()['gripper']
                        holding_position = args.holding_position
                        if holding_position is None and gripper.get('connected') and gripper.get('status') == 2:
                            holding_position = client.gripper_snapshot(
                                stable_samples=5, expected_status=2)['state']['position']
                        if holding_position is not None:
                            packet = client.verify_holding(holding_position)
                            emit('holding_verified', when='before_motion', gripper=packet)
                        last_gripper_check = 0.

                        def check_feedback():
                            nonlocal last_gripper_check
                            check_control()
                            if holding_position is not None and time.monotonic()-last_gripper_check >= .15:
                                feedback = client.gripper_snapshot(
                                    stable_samples=1, expected_status=2,
                                    expected_position=holding_position)
                                last_gripper_check = time.monotonic()
                                emit('holding_feedback', gripper=feedback['state'])

                        started = time.monotonic()
                        result = execute(robot, sdk, route, segments, limits, check_feedback, emit)
                        emit('execution_complete', elapsed_s=time.monotonic()-started)
                        if holding_position is not None:
                            check_control()
                            packet = client.verify_holding(holding_position)
                            emit('holding_verified', when='after_motion', gripper=packet)
                            result['holding_verified'] = True
                            result['gripper'] = packet
                        result['motion_sent'] = True
                    result['final_power'] = str(call(robot.powerState))
                finally:
                    call(robot.disconnectFromRobot)
                    emit('disconnected')
            result['log_path'] = str(log_path)
            write_json(output, result)
        except BaseException as exc:
            emit('error', message=str(exc))
            print(f'Route failed; inspect {log_path}; do not retry execution blindly', file=sys.stderr)
            raise


if __name__ == '__main__':
    main()

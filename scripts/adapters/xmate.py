"""xMate tools and contract checks; imports do not connect devices or load SDKs."""

from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import tempfile
import uuid

from scripts.runtime.contract_io import load_json
from scripts.runtime.tool_registry import ToolRegistry
from scripts.runtime.worker_process import run_worker

ROOT = Path(__file__).resolve().parents[2]


def _joint_route(args, operation):
    key = {'check': 'route_path', 'plan': 'path_spec', 'batch': 'batch_spec',
           'execute': 'checked_route_path'}[operation]
    folder = ROOT / ('logs/runs' if operation == 'execute' else 'captures')
    folder.mkdir(parents=True, exist_ok=True)
    output = folder / ('joint-route-' + uuid.uuid4().hex + '.json')
    # The worker owns the lease and monitors it during motion. Do not kill an
    # active SDK process with a parent-side subprocess timeout.
    command = [str(ROOT / '.venv/bin/python'), '-m', 'scripts.hardware.joint_route',
                              operation, str(Path(args[key]).resolve()), '--output', str(output),
                              '--web-url', args.get('web_url', 'http://127.0.0.1:8080')]
    if operation == 'execute' and 'holding_position' in args:
        command.extend(['--holding-position', str(args['holding_position'])])
    process = run_worker(command, cwd=ROOT)
    if process.returncode:
        raise RuntimeError('Joint route failed: ' + (process.stderr or process.stdout)[-3000:])
    # Sampled FK data may exceed the external request-size limit.
    with output.open() as stream:
        result = json.load(stream)
    if operation not in ('execute', 'batch'):
        return {'status': result['status'], 'checked_route_path': str(output),
                'log_path': result['log_path'], 'motion_sent': False,
                'segment_count': len(result['segments'])}
    return result


def check_joint_route(args):
    return _joint_route(args, 'check')


def execute_joint_route(args):
    return _joint_route(args, 'execute')


def plan_joint_route(args):
    return _joint_route(args, 'plan')


def plan_joint_routes(args):
    return _joint_route(args, 'batch')


def robot_state(args):
    from scripts.hardware.sensor_client import SensorClient

    client = SensorClient(args.get('web_url', 'http://127.0.0.1:8080'))
    # Hold existing web ownership while the separate SDK process queries the idle arm.
    with client.control(), tempfile.TemporaryDirectory(prefix='robot-state-') as folder:
        output = Path(folder) / 'state.json'
        process = subprocess.run([str(ROOT / '.venv/bin/python'),
                                  '-m', 'scripts.hardware.check_connection', '--output', str(output)],
                                 cwd=ROOT, capture_output=True, text=True, timeout=45)
        if process.returncode:
            raise RuntimeError('Robot state query failed: ' + (process.stderr or process.stdout)[-2000:])
        result = load_json(output)
        # Keep the tool response focused; full configuration/history remains in its log.
        for key in ('saved_tools', 'recent_warning_error_logs'):
            result['values'].pop(key, None)
        return result


def move_gripper(args):
    from scripts.hardware.sensor_client import SensorClient

    with SensorClient(args.get('web_url', 'http://127.0.0.1:8080')).control() as client:
        return client.action('gripper_target', target=args['target_position'])


def close_gripper(args):
    from scripts.hardware.sensor_client import SensorClient

    with SensorClient(args.get('web_url', 'http://127.0.0.1:8080')).control() as client:
        return client.action('grasp_close', target_position=args['target_position'])


def observe(args):
    from scripts.vision.capture import capture_resident
    from scripts.hardware.sensor_client import SensorClient

    client = SensorClient(args.get('web_url', 'http://127.0.0.1:8080'))
    result = {'origin': 'live'}
    for source in args['sources']:
        if source == 'rgb':
            result['rgb'] = capture_resident(client=client, after=args.get('after', 0.))
        elif source == 'gripper':
            result['gripper'] = client.gripper_snapshot(after=args.get('after', 0.))
        else:
            raise ValueError(f'Unsupported observation source: {source}')
    result['collected_at'] = datetime.now(timezone.utc).isoformat()
    return result


def route_errors(plan):
    consumed = set()
    errors = []
    for index, stage in enumerate(plan['stages']):
        if stage['skill'] == 'execute_joint_route':
            if 'checked_route_path' in stage.get('args_from', {}):
                key = ('ref', stage['args_from']['checked_route_path'])
            else:
                key = ('joint', str(Path(stage['args']['checked_route_path']).resolve()))
            if key in consumed:
                errors.append({'path': f'/stages/{index}/args/checked_route_path',
                               'message': 'Checked joint route already consumed in this plan'})
            consumed.add(key)
            continue
    return errors


def create_registry():
    path = Path(__file__).resolve().parents[2] / 'config/capabilities/xmate.json'
    return ToolRegistry(load_json(path), handlers={'observe': observe, 'robot_state': robot_state,
                        'move_gripper': move_gripper,
                        'close_gripper': close_gripper,
                        'check_joint_route': check_joint_route,
                        'plan_joint_route': plan_joint_route,
                        'plan_joint_routes': plan_joint_routes,
                        'execute_joint_route': execute_joint_route},
                        plan_check=route_errors)

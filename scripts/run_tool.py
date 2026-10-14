"""Invoke one registered tool with JSON arguments and print its actual result."""

import argparse
import json
import sys
from pathlib import Path

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.adapters import load_adapter
from scripts.runtime.contract_io import load_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('tool')
    parser.add_argument('--adapter', required=True)
    parser.add_argument('--args', default='-', help='JSON file, or - for stdin (default)')
    args = parser.parse_args()
    try:
        registry = load_adapter(args.adapter)
        result = registry.invoke(args.tool, load_json(args.args))
        print(json.dumps({'adapter_id': registry.adapter_id, 'tool': args.tool,
                          'result': result}, indent=2, allow_nan=False))
        return 0
    except Exception as exc:
        print(json.dumps({'tool': args.tool, 'error': str(exc)}), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())

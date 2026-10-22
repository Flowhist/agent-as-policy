"""Explicit capability registration and tool calls; no dynamic code loading."""

from copy import deepcopy
from pathlib import Path

from scripts.runtime.contract_io import load_json, numeric_errors, pointer, schema_validator

ROOT = Path(__file__).resolve().parents[2]


class ToolRegistry:
    def __init__(self, manifest, *, handlers=None, argument_checks=None, plan_check=None):
        self._manifest = deepcopy(manifest)
        validator = schema_validator(load_json(ROOT / 'config/capabilities.schema.json'))
        errors = list(validator.iter_errors(self._manifest))
        if errors:
            raise ValueError(f'Invalid capability manifest: {errors[0].message}')
        self._handlers = dict(handlers or {})
        self._argument_checks = dict(argument_checks or {})
        self._plan_check = plan_check
        self._inputs, self._outputs = {}, {}
        definitions = self._manifest['tools']
        if (self._handlers.keys() | self._argument_checks.keys()) - definitions.keys():
            raise ValueError('Registered implementation/check has no tool definition')
        if any(not callable(fn) for fn in (*self._handlers.values(), *self._argument_checks.values())):
            raise ValueError('Handlers and argument checks must be trusted callables')
        if plan_check is not None and not callable(plan_check):
            raise ValueError('Plan check must be a trusted callable')
        for name, definition in definitions.items():
            self._inputs[name] = schema_validator(definition['input_schema'])
            output = definition['output_schema']
            self._outputs[name] = schema_validator(output) if output is not None else None
            if definition['implementation'] == 'implemented':
                if name not in self._handlers or output is None:
                    raise ValueError(f'Implemented tool {name} needs a handler and output schema')
            elif name in self._handlers:
                raise ValueError(f'Proposed tool {name} cannot register a handler')

    @property
    def adapter_id(self):
        return self._manifest['adapter_id']

    @property
    def version(self):
        return self._manifest['version']

    def describe(self, name=None):
        manifest = deepcopy(self._manifest)
        if name is not None:
            if name not in manifest['tools']:
                raise ValueError(f'Unknown tool: {name}')
            manifest['tools'] = {name: manifest['tools'][name]}
        return manifest

    def unavailable_tools(self, names):
        return sorted({name for name in names if name not in self._handlers})

    def argument_errors(self, name, args):
        if name not in self._inputs:
            return [{'path': '', 'message': f'Unknown tool: {name}'}]
        errors = list(numeric_errors(args))[:32]
        if errors:
            return errors
        for error in self._inputs[name].iter_errors(args):
            errors.append({'path': pointer(error.absolute_path), 'message': error.message[:800]})
            if len(errors) >= 32:
                break
        if not errors and name in self._argument_checks:
            errors.extend(self._argument_checks[name](args))
        return errors

    def plan_errors(self, plan):
        errors = []
        if plan['adapter_id'] != self.adapter_id:
            errors.append({'path': '/adapter_id', 'message': 'Plan does not match selected adapter'})
        if plan['capability_version'] != self.version:
            errors.append({'path': '/capability_version', 'message': 'Capability version mismatch'})
        for index, stage in enumerate(plan['stages']):
            if stage['skill'] not in self._inputs:
                errors.append({'path': f'/stages/{index}/skill', 'message': 'Unknown tool'})
                continue
            bindings = stage.get('args_from', {})
            schema = deepcopy(self._manifest['tools'][stage['skill']]['input_schema'])
            for key in bindings:
                if key in stage['args'] or key not in schema.get('properties', {}):
                    errors.append({'path': f'/stages/{index}/args_from/{key}',
                                   'message': 'Bound argument must be a declared parameter absent from args'})
            # Validate literal fields now; invoke validates resolved values.
            if bindings:
                schema['required'] = [k for k in schema.get('required', []) if k not in bindings]
                argument_errors = [{'path': pointer(e.absolute_path), 'message': e.message}
                                   for e in schema_validator(schema).iter_errors(stage['args'])]
            else:
                argument_errors = self.argument_errors(stage['skill'], stage['args'])
            for error in argument_errors:
                errors.append({'path': f'/stages/{index}/args' + error['path'],
                               'message': error['message']})
        if not errors and self._plan_check is not None:
            errors.extend(self._plan_check(plan))
        return errors

    def result_errors(self, name, result):
        """Validate a stored or newly returned result without invoking the tool."""
        if name not in self._outputs or self._outputs[name] is None:
            return [{'path': '', 'message': 'Tool has no registered output schema'}]
        errors = list(numeric_errors(result))[:32]
        if not errors:
            for error in self._outputs[name].iter_errors(result):
                errors.append({'path': pointer(error.absolute_path), 'message': error.message[:800]})
                if len(errors) >= 32:
                    break
        return errors

    def invoke(self, name, args):
        """Validate arguments, call the installed adapter, and validate its result."""
        errors = self.argument_errors(name, args)
        if errors:
            raise ValueError(f'Invalid tool arguments: {errors}')
        if name not in self._handlers:
            raise ValueError('Tool is not implemented')
        result = self._handlers[name](deepcopy(args))
        errors = self.result_errors(name, result)
        if errors:
            raise ValueError(f'Tool {name} returned an invalid result: {errors[0]}')
        return result

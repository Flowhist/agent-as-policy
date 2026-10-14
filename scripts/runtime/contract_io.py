"""Strict offline JSON and local-only Schema helpers; no device imports."""

import json
import math
from pathlib import Path
import sys

from jsonschema import Draft202012Validator
from referencing import Registry, Resource
from referencing.exceptions import Unresolvable

MAX_BYTES = 1024 * 1024


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def finite_float(text):
    value = float(text)
    if not math.isfinite(value):
        raise ValueError("JSON numbers must be finite")
    return value


def reject_constant(text):
    raise ValueError(f"Non-standard JSON constant: {text}")


def load_json(path):
    if path == "-":
        raw = sys.stdin.buffer.read(MAX_BYTES + 1)
    else:
        with Path(path).open("rb") as stream:
            raw = stream.read(MAX_BYTES + 1)
    if len(raw) > MAX_BYTES:
        raise ValueError("JSON input exceeds 1 MiB")
    return json.loads(raw.decode("utf-8"), object_pairs_hook=unique_object,
                      parse_float=finite_float, parse_constant=reject_constant)


def local_schema_refs(value):
    # Keep schema edits from introducing network retrieval into an offline tool.
    refs = []
    if isinstance(value, dict):
        for key, child in value.items():
            if key in ("$ref", "$dynamicRef"):
                if not isinstance(child, str) or not child.startswith("#"):
                    raise ValueError("Only local schema references are allowed")
                refs.append(child)
            refs.extend(local_schema_refs(child))
    elif isinstance(value, list):
        for child in value:
            refs.extend(local_schema_refs(child))
    return refs


def pointer(parts):
    return "".join("/" + str(part).replace("~", "~0").replace("/", "~1")
                   for part in parts)


def numeric_errors(value, parts=()):
    if isinstance(value, dict):
        for key, child in value.items():
            yield from numeric_errors(child, (*parts, key))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from numeric_errors(child, (*parts, index))
    elif type(value) in (int, float):
        try:
            finite = math.isfinite(value)
        except OverflowError:
            finite = False
        if not finite:
            yield {"path": pointer(parts),
                   "message": "Numbers must be representable as finite native values"}


def schema_validator(schema):
    if (not isinstance(schema, dict)
            or schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema"):
        raise ValueError("Contracts must use JSON Schema draft 2020-12")
    refs = local_schema_refs(schema)
    Draft202012Validator.check_schema(schema)
    if list(numeric_errors(schema)):
        raise ValueError("Schema numbers must be finite")

    def reject_nested_ids(value, root=False):
        if isinstance(value, dict):
            if not root and "$id" in value:
                raise ValueError("Nested schema IDs are not supported")
            for child in value.values():
                reject_nested_ids(child)
        elif isinstance(value, list):
            for child in value:
                reject_nested_ids(child)

    reject_nested_ids(schema, root=True)
    uri = schema.get("$id", "urn:agent-as-policy:local-schema")
    registry = Registry().with_resource(uri, Resource.from_contents(schema))
    resolver = registry.resolver(uri)
    for ref in refs:
        try:
            resolver.lookup(ref)
        except Unresolvable as exc:
            raise ValueError(f"Unresolvable local schema reference: {ref}") from exc
    return Draft202012Validator(schema, registry=registry)

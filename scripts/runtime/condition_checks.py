"""Small local comparator for actual tool values; no state store or model calls."""

import math
import operator


def scalar_type(value):
    if type(value) in (int, float):
        try:
            return 'number' if math.isfinite(value) else None
        except OverflowError:
            return None
    if type(value) is bool:
        return 'boolean'
    if isinstance(value, str):
        return 'string'
    return None


def evaluate_condition(condition, values):
    """Return pass/fail/unknown. Caller supplies fresh, tool-produced ref -> value."""
    actual = values.get(condition['ref'])
    expected, op = condition['value'], condition['op']
    category = scalar_type(actual)
    if category is None:
        return 'unknown'
    if op == 'in':
        if not isinstance(expected, list) or not expected:
            return 'unknown'
        candidates = expected
    else:
        candidates = [expected]
    if any(scalar_type(value) != category for value in candidates):
        return 'unknown'
    if op in ('lt', 'le', 'gt', 'ge') and category != 'number':
        return 'unknown'
    comparisons = dict(eq=operator.eq, ne=operator.ne, lt=operator.lt,
                       le=operator.le, gt=operator.gt, ge=operator.ge)
    if op == 'in':
        passed = actual in candidates
    elif op in comparisons:
        passed = comparisons[op](actual, expected)
    else:
        return 'unknown'
    return 'pass' if passed else 'fail'

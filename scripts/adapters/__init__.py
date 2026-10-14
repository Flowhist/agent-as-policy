"""Trusted adapter registration; plans and manifests cannot import code."""

from . import xmate

ADAPTERS = {'xmate': xmate.create_registry}


def load_adapter(name):
    if name not in ADAPTERS:
        raise ValueError(f'Unknown installed adapter: {name}')
    return ADAPTERS[name]()

from typing import Callable, Dict, Any
from importlib import import_module

registry: Dict[str, Dict[str, Any]] = {}

def _register(name: str, summary: str, run: Callable[[dict], int]):
    registry[name] = {"summary": summary, "run": run}

# existing demos (imitation)...
def _run_imitation(kwargs: dict) -> int:
    mod = import_module("pib_sdk.demos.imitation.demo")
    return mod.main(kwargs)

_register(
    "imitation",
    "Mirror the user’s hands to pib (arms + fingers) using OAK-D-Lite.",
    _run_imitation,
)

# NEW: color_follow (OAK-only)
def _run_color_follow(kwargs: dict) -> int:
    mod = import_module("pib_sdk.demos.color_follow.color_follow")
    return mod.main(kwargs)

_register(
    "color_follow",
    "Track a colored object (blue/pink/green/orange/red) with OAK-D-Lite and follow it with the head.",
    _run_color_follow,
)

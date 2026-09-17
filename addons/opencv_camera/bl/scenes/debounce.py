"""Debounced timers for the scenes.

Dragging a slider fires an ``update`` callback per mouse move, so the scene
rebuild has to wait until the user stops.  This is the same ``bpy.app.timers``
pattern ``bl/preview.py`` uses for its preview render, generalised to one
independent pending callback per key (scene id).

Timers do not fire in background mode, so ``schedule`` is a no-op there.
"""

from __future__ import annotations

import time
from typing import Callable, Dict

import bpy

#: debounce window in seconds (matches the preview render's 0.6 s feel but is
#: a rebuild, not a render, so it can be a bit snappier)
DEBOUNCE_SECONDS = 0.2

#: key -> {"time": float, "callback": callable, "delay": float, "registered": bool}
_PENDING: Dict[str, Dict] = {}


def schedule(key: str, callback: Callable[[], None],
             delay: float = DEBOUNCE_SECONDS) -> None:
    """Run ``callback`` once ``key`` stops being scheduled (UI sessions only)."""
    if bpy.app.background:
        return
    entry = _PENDING.setdefault(key, {"registered": False})
    entry["time"] = time.time()
    entry["callback"] = callback
    entry["delay"] = max(0.0, float(delay))
    if not entry.get("registered"):
        entry["registered"] = True
        bpy.app.timers.register(lambda: _tick(key), first_interval=entry["delay"])


def cancel(key: str) -> None:
    """Forget a pending callback."""
    entry = _PENDING.get(key)
    if entry is not None:
        entry["callback"] = None
        entry["time"] = 0.0


def pending(key: str) -> bool:
    entry = _PENDING.get(key)
    return bool(entry and entry.get("callback") is not None)


def _tick(key: str):
    entry = _PENDING.get(key)
    if entry is None or entry.get("callback") is None:
        if entry is not None:
            entry["registered"] = False
        return None
    if time.time() - float(entry["time"]) < float(entry["delay"]):
        return min(0.1, float(entry["delay"]))  # still editing, check again
    callback = entry["callback"]
    cancel(key)
    try:
        callback()
    except Exception:  # never let a timer callback raise into the UI loop
        pass
    entry["registered"] = False
    return None

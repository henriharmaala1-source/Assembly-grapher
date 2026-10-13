"""
Simple publish/subscribe event bus for decoupled panel communication.

Usage:
    bus = EventBus()
    bus.subscribe("run_dfma", my_callback)   # callback(data)
    bus.publish("run_dfma", payload)
"""

import traceback
from collections import defaultdict
from typing import Any, Callable


class EventBus:
    def __init__(self) -> None:
        self._listeners: dict[str, list[Callable]] = defaultdict(list)

    def subscribe(self, event: str, callback: Callable[[Any], None]) -> None:
        self._listeners[event].append(callback)

    def unsubscribe(self, event: str, callback: Callable) -> None:
        self._listeners[event] = [
            cb for cb in self._listeners[event] if cb is not callback
        ]

    def publish(self, event: str, data: Any = None) -> None:
        # One failing subscriber must not stop the others from receiving the
        # event — otherwise app state silently never updates.
        for cb in list(self._listeners[event]):
            try:
                cb(data)
            except Exception:
                traceback.print_exc()

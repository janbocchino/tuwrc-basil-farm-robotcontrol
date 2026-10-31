"""Thread-safe aggregation of partial ROS JointState messages."""

from __future__ import annotations

import math
import threading
import time


class JointStateStore:
    """Merge partial joint updates while making stale data explicit."""

    def __init__(self, stale_after_sec: float = 2.0):
        self._lock = threading.Lock()
        self._latest_received_at: float | None = None
        self._joints: dict[str, dict] = {}
        self._stale_after_sec = float(stale_after_sec)

    def update(self, message) -> None:
        received_at = time.time()
        updates = {}
        for index, name in enumerate(message.name):
            position = self._value_at(message.position, index)
            velocity = self._value_at(message.velocity, index)
            effort = self._value_at(message.effort, index)
            updates[str(name)] = {
                "name": str(name),
                "position": position,
                "position_deg": (
                    math.degrees(position) if position is not None else None
                ),
                "velocity": velocity,
                "effort": effort,
                "received_at": received_at,
            }

        with self._lock:
            self._joints.update(updates)
            self._latest_received_at = received_at

    def snapshot(self):
        now = time.time()
        with self._lock:
            if self._latest_received_at is None:
                return None
            joints = []
            for joint in self._joints.values():
                item = dict(joint)
                age_ms = int((now - item.pop("received_at")) * 1000)
                item["age_ms"] = age_ms
                item["stale"] = age_ms > self._stale_after_sec * 1000
                joints.append(item)
            return {
                "received_at": self._latest_received_at,
                "age_ms": int((now - self._latest_received_at) * 1000),
                "joints": joints,
            }

    def joint_positions(self):
        with self._lock:
            return {
                name: joint["position"]
                for name, joint in self._joints.items()
                if joint["position"] is not None
            }

    @staticmethod
    def _value_at(values, index):
        if index >= len(values):
            return None
        value = float(values[index])
        return None if math.isnan(value) else value

"""Parsing helpers for the Arduino rail serial protocol."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RailStatus:
    lm_left: bool
    lm_right: bool
    position_steps: int
    homed: bool


def parse_status_line(line: str) -> RailStatus | None:
    if not line.startswith("ST "):
        return None
    values = line[3:].split()
    if len(values) != 4:
        return None
    try:
        position_steps = int(values[2])
    except ValueError:
        return None
    if any(value not in {"0", "1"} for value in (values[0], values[1], values[3])):
        return None
    return RailStatus(
        lm_left=values[0] == "1",
        lm_right=values[1] == "1",
        position_steps=position_steps,
        homed=values[3] == "1",
    )

#!/usr/bin/env python3
"""Print six ST3215 raw positions directly in the terminal.

This tool is intentionally boring and safe: it only opens the serial bus and
calls ReadPosition. It does not start torque, set modes, set speeds, or write
target positions.
"""

from __future__ import annotations

import argparse
import glob
import sys
import time
from pathlib import Path


def import_st3215():
    try:
        from st3215 import ST3215
        return ST3215
    except ImportError:
        local_library = Path.home() / "libs" / "python-st3215"
        if local_library.is_dir():
            sys.path.insert(0, str(local_library))
            from st3215 import ST3215
            return ST3215
        raise


ST3215 = import_st3215()


def find_port(configured_port: str) -> str:
    if configured_port:
        return configured_port
    candidates = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if not candidates:
        raise RuntimeError("Kein /dev/ttyACM* oder /dev/ttyUSB* gefunden")
    return candidates[0]


def parse_ids(value: str) -> list[int]:
    ids = [int(part.strip()) for part in value.split(",") if part.strip()]
    if not ids:
        raise argparse.ArgumentTypeError("Mindestens eine Servo-ID angeben")
    if len(set(ids)) != len(ids):
        raise argparse.ArgumentTypeError("Servo-IDs dürfen nicht doppelt sein")
    return ids


def format_position(value) -> str:
    if value is None:
        return "FEHLER"
    return f"{int(value):4d}"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "ST3215-Positionen direkt anzeigen. Dieses Programm liest nur "
            "und sendet keine Bewegungsbefehle."
        )
    )
    parser.add_argument("--port", default="", help="z.B. /dev/ttyACM0")
    parser.add_argument(
        "--ids",
        type=parse_ids,
        default=[1, 2, 3, 4, 5, 6],
        help="Kommagetrennte Servo-IDs, Standard: 1,2,3,4,5,6",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=5.0,
        help="Ausgaben pro Sekunde, Standard: 5",
    )
    args = parser.parse_args()

    if args.rate <= 0.0:
        raise RuntimeError("--rate muss größer als 0 sein")

    port = find_port(args.port)
    servo = ST3215(port)
    print(f"Passives Auslesen auf {port}; IDs={args.ids}")
    print("Dieses Programm sendet keine Bewegungsbefehle.")
    print("Abbrechen mit Ctrl+C.\n")

    try:
        while True:
            values = [servo.ReadPosition(servo_id) for servo_id in args.ids]
            line = " | ".join(
                f"ID {servo_id}: {format_position(value)}"
                for servo_id, value in zip(args.ids, values)
            )
            print(line, flush=True)
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        print("\nBeendet.")
        return 0
    finally:
        try:
            servo.portHandler.closePort()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())

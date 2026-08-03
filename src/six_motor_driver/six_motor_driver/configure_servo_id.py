#!/usr/bin/env python3
"""Assign a unique bus ID to one individually connected ST3215 servo."""

from __future__ import annotations

import argparse
import glob
import sys
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


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "ID eines einzeln angeschlossenen ST3215 ändern. "
            "Währenddessen darf nur dieser eine Servo am Bus hängen."
        )
    )
    parser.add_argument("--port", default="", help="z. B. /dev/ttyACM0")
    parser.add_argument("--old-id", type=int, default=1)
    parser.add_argument("--new-id", type=int, required=True)
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Sicherheitsabfrage überspringen",
    )
    args = parser.parse_args()

    if not 0 <= args.old_id <= 253 or not 0 <= args.new_id <= 253:
        parser.error("IDs müssen zwischen 0 und 253 liegen")
    if args.old_id == args.new_id:
        parser.error("--old-id und --new-id sind identisch")

    port = find_port(args.port)
    print("ACHTUNG: Es darf jetzt nur EIN Servo am seriellen Bus hängen.")
    print(f"Geplante Änderung auf {port}: ID {args.old_id} -> {args.new_id}")
    if not args.yes:
        answer = input("Zum Ändern CHANGE eingeben: ").strip()
        if answer != "CHANGE":
            print("Abgebrochen.")
            return 0

    servo = ST3215(port)
    try:
        if not servo.PingServo(args.old_id):
            raise RuntimeError(f"Servo ID {args.old_id} antwortet nicht")
        if servo.PingServo(args.new_id):
            raise RuntimeError(
                f"ID {args.new_id} antwortet bereits; Änderung abgebrochen"
            )

        error = servo.ChangeId(args.old_id, args.new_id)
        if error is not None:
            raise RuntimeError(str(error))
        if not servo.PingServo(args.new_id):
            raise RuntimeError(
                "ID wurde geschrieben, antwortet danach aber nicht"
            )
        if servo.LockEprom(args.new_id) != 0:
            raise RuntimeError(
                "ID wurde geändert, aber das EEPROM konnte nicht gesperrt werden"
            )
        print(f"Erfolg: Servo antwortet jetzt als ID {args.new_id}.")
        return 0
    except Exception as error:
        print(f"Fehler: {error}", file=sys.stderr)
        return 1
    finally:
        try:
            servo.portHandler.closePort()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())

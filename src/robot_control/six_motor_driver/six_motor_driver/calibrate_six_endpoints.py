#!/usr/bin/env python3
"""Interactively capture six cyclic ST3215 endpoint arcs."""

from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path

import yaml

from .cyclic_calibration import STEPS_PER_REVOLUTION, forward_distance
from .six_motor_position_reader import ST3215


def find_port(configured: str) -> str:
    if configured:
        return configured
    candidates = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if not candidates:
        raise RuntimeError("Kein serieller ST3215-Port gefunden")
    return candidates[0]


def default_config() -> Path:
    from ament_index_python.packages import get_package_share_directory

    return (
        Path(get_package_share_directory("six_motor_driver"))
        / "config"
        / "six_motor_calibration.yaml"
    )


def read_required(servo, servo_id: int) -> int:
    value = servo.ReadPosition(servo_id)
    if value is None:
        raise RuntimeError(f"Position von Servo ID {servo_id} nicht lesbar")
    return int(value)


def capture(servo, servo_id: int, prompt: str) -> int:
    input(prompt)
    samples = [read_required(servo, servo_id) for _ in range(5)]
    samples.sort()
    value = samples[len(samples) // 2]
    print(f"  Erfasst: {value}")
    return value


def point_on_arc(start: int, end: int, point: int) -> bool:
    """Return true if point lies on the positive cyclic arc start -> end."""
    return forward_distance(start, point) <= forward_distance(start, end)


def choose_allowed_arc(
    servo_id: int,
    endpoint_a: int,
    endpoint_b: int,
    inside_point: int,
) -> tuple[int, int]:
    """Choose the endpoint order whose cyclic arc contains inside_point."""
    if endpoint_a == endpoint_b:
        raise RuntimeError(
            f"ID {servo_id}: Die beiden Maximalpositionen sind identisch"
        )
    if inside_point in (endpoint_a, endpoint_b):
        raise RuntimeError(
            f"ID {servo_id}: Die Zwischenposition darf nicht exakt auf "
            "einer Maximalposition liegen"
        )

    a_to_b = point_on_arc(endpoint_a, endpoint_b, inside_point)
    b_to_a = point_on_arc(endpoint_b, endpoint_a, inside_point)
    if a_to_b == b_to_a:
        raise RuntimeError(
            f"ID {servo_id}: Zwischenposition {inside_point} konnte den "
            "erlaubten Drehbogen nicht eindeutig bestimmen"
        )

    lower = endpoint_a if a_to_b else endpoint_b
    upper = endpoint_b if a_to_b else endpoint_a
    span = forward_distance(lower, upper)
    if span < 50:
        raise RuntimeError(
            f"ID {servo_id}: Bereich mit {span} Schritten zu klein"
        )
    return lower, upper


def apply_margin(
    servo_id: int,
    lower: int,
    upper: int,
    margin_steps: int,
) -> tuple[int, int]:
    span = forward_distance(lower, upper)
    if margin_steps < 0:
        raise RuntimeError("--margin-steps darf nicht negativ sein")
    if span <= 2 * margin_steps + 20:
        raise RuntimeError(
            f"ID {servo_id}: Bereich mit {span} Schritten ist zu klein "
            f"für {margin_steps} Schritte Sicherheitsabstand pro Seite"
        )
    safe_lower = (lower + margin_steps) % STEPS_PER_REVOLUTION
    safe_upper = (
        lower + span - margin_steps
    ) % STEPS_PER_REVOLUTION
    return safe_lower, safe_upper


def midpoint(lower: int, upper: int) -> int:
    span = forward_distance(lower, upper)
    return (lower + int(round(span / 2.0))) % STEPS_PER_REVOLUTION


def capture_zero_or_midpoint(
    servo,
    servo_id: int,
    safe_lower: int,
    safe_upper: int,
) -> int:
    middle = midpoint(safe_lower, safe_upper)
    answer = input(
        "  Für RViz-Abgleich: Servo an die Position bewegen, die in RViz "
        "'0 rad' sein soll, dann Enter.\n"
        f"  Oder MITTE eingeben, um die sichere Mitte {middle} zu verwenden: "
    ).strip()
    if answer.upper() == "MITTE":
        print(f"  Nullposition: sichere Mitte {middle}")
        return middle
    samples = [read_required(servo, servo_id) for _ in range(5)]
    samples.sort()
    value = samples[len(samples) // 2]
    if not point_on_arc(safe_lower, safe_upper, value):
        raise RuntimeError(
            f"ID {servo_id}: RViz-Nullposition {value} liegt nicht im "
            f"sicheren Bereich {safe_lower} -> {safe_upper}"
        )
    print(f"  Nullposition für RViz 0 rad: {value}")
    return value


def selected_indices(selection: str, servo_ids: list[int], names: list[str]) -> list[int]:
    if not selection:
        return list(range(len(servo_ids)))
    indices: list[int] = []
    for part in selection.split(","):
        token = part.strip()
        if not token:
            continue
        index = None
        if token in names:
            index = names.index(token)
        else:
            try:
                number = int(token)
            except ValueError as error:
                raise RuntimeError(
                    f"Auswahl {token!r} ist weder Name noch Nummer"
                ) from error
            if number in servo_ids:
                index = servo_ids.index(number)
            elif 1 <= number <= len(servo_ids):
                index = number - 1
        if index is None:
            raise RuntimeError(
                f"Auswahl {token!r} passt zu keiner Servo-ID, keinem "
                "Index und keinem Namen"
            )
        if index not in indices:
            indices.append(index)
    if not indices:
        raise RuntimeError("Keine Servos zur Kalibrierung ausgewählt")
    return indices


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Mechanische Maximalpositionen der Servo-IDs 1 bis 6 erfassen, "
            "mit einer Zwischenposition den erlaubten Drehbogen bestimmen "
            "und sichere Grenzen sowie RViz-Nullpositionen speichern."
        )
    )
    parser.add_argument("--port", default="")
    parser.add_argument("--config", type=Path, default=default_config())
    parser.add_argument(
        "--only",
        default="",
        help=(
            "Nur einzelne Servos kalibrieren, z.B. --only 3 oder "
            "--only 1,4,6. Akzeptiert Servo-ID, Index oder Namen."
        ),
    )
    parser.add_argument(
        "--margin-steps",
        type=int,
        default=10,
        help=(
            "Sicherheitsabstand zu jeder mechanischen Maximalposition. "
            "Standard: 10 Schritte."
        ),
    )
    parser.add_argument(
        "--zero-only",
        action="store_true",
        help=(
            "Nur die RViz-Nullposition der ausgewählten Servos neu erfassen; "
            "bestehende Endgrenzen bleiben unverändert."
        ),
    )
    args = parser.parse_args()

    port = find_port(args.port)
    document = yaml.safe_load(args.config.read_text())
    node_config = document.get("/**") or document.get(
        "six_motor_position_reader"
    )
    parameters = node_config["ros__parameters"]
    servo_ids = [int(value) for value in parameters["servo_ids"]]
    names = [str(value) for value in parameters["names"]]
    lower_values = [int(value) for value in parameters["lower_limits_steps"]]
    upper_values = [int(value) for value in parameters["upper_limits_steps"]]
    middle_values = [int(value) for value in parameters["zero_positions_steps"]]
    indices = selected_indices(args.only, servo_ids, names)

    print("ENDPUNKT-KALIBRIERUNG FÜR SECHS ST3215")
    print("Die Motoren werden nicht bewegt. Das Drehmoment wird ausgeschaltet.")
    print(
        "Pro Servo erfasst du zwei Maximalpositionen und danach irgendeine "
        "Position dazwischen. Daraus wird automatisch bestimmt, welcher "
        "zyklische Weg erlaubt ist."
    )
    print(
        f"Gespeichert wird der sichere Bereich mit {args.margin_steps} "
        "Schritten Abstand zu beiden Maximalpositionen."
    )
    print(
        "Zusätzlich wird pro Servo eine RViz-Nullposition erfasst. Diese "
        "Position entspricht danach 0 rad im digitalen Modell."
    )
    print(
        "Ausgewählt: "
        + ", ".join(
            f"Servo {index + 1}/ID {servo_ids[index]}/Name {names[index]}"
            for index in indices
        )
    )
    print("Bei Unsicherheit zuerst mit dem passiven Positionsleser beobachten.")
    if input("Zum Start CALIBRATE eingeben: ").strip() != "CALIBRATE":
        print("Abgebrochen.")
        return 0

    servo = ST3215(port)
    try:
        for index in indices:
            servo_id = servo_ids[index]
            if not servo.PingServo(servo_id):
                raise RuntimeError(f"Servo ID {servo_id} antwortet nicht")
            servo.StopServo(servo_id)

        for index in indices:
            servo_id = servo_ids[index]
            print(f"\nServo {index + 1} / ID {servo_id} / Name {names[index]}")
            if args.zero_only:
                zero = capture_zero_or_midpoint(
                    servo,
                    servo_id,
                    lower_values[index],
                    upper_values[index],
                )
                middle_values[index] = zero
                print(
                    f"  Endgrenzen bleiben unverändert: "
                    f"{lower_values[index]} -> {upper_values[index]}"
                )
                print(f"  Neue RViz-Nullposition: {zero}")
                continue

            endpoint_a = capture(
                servo,
                servo_id,
                "  Von Hand an die ERSTE Maximalposition bewegen, dann Enter: ",
            )
            endpoint_b = capture(
                servo,
                servo_id,
                "  Von Hand an die ZWEITE Maximalposition bewegen, dann Enter: ",
            )
            inside_point = capture(
                servo,
                servo_id,
                "  Von Hand an IRGENDEINE sichere Position DAZWISCHEN "
                "bewegen, dann Enter: ",
            )
            physical_lower, physical_upper = choose_allowed_arc(
                servo_id,
                endpoint_a,
                endpoint_b,
                inside_point,
            )
            safe_lower, safe_upper = apply_margin(
                servo_id,
                physical_lower,
                physical_upper,
                args.margin_steps,
            )
            zero = capture_zero_or_midpoint(
                servo,
                servo_id,
                safe_lower,
                safe_upper,
            )
            physical_span = forward_distance(physical_lower, physical_upper)
            safe_span = forward_distance(safe_lower, safe_upper)
            lower_values[index] = safe_lower
            upper_values[index] = safe_upper
            middle_values[index] = zero
            print(
                f"  Maximalpositionen: {endpoint_a} und {endpoint_b}"
            )
            print(
                f"  Zwischenposition {inside_point} liegt auf dem erlaubten "
                f"Weg: {physical_lower} -> {physical_upper}"
            )
            print(
                f"  Physische Länge={physical_span} Schritte"
            )
            print(
                f"  Gespeicherter sicherer Weg: {safe_lower} -> "
                f"{safe_upper}, Länge={safe_span} Schritte"
            )
            print(
                f"  Gespeicherte RViz-Nullposition: {zero}"
            )

        parameters["lower_limits_steps"] = lower_values
        parameters["upper_limits_steps"] = upper_values
        parameters["zero_positions_steps"] = middle_values

        backup = args.config.with_suffix(args.config.suffix + ".bak")
        backup.write_text(args.config.read_text())
        args.config.write_text(
            yaml.safe_dump(document, sort_keys=False, allow_unicode=True)
        )
        print(f"\nKalibrierung gespeichert: {args.config}")
        print(f"Sicherung der vorherigen Datei: {backup}")
        print("Als Nächstes neu bauen und den Sechs-Slider-Modus starten.")
        return 0
    finally:
        for servo_id in servo_ids:
            try:
                servo.StopServo(servo_id)
            except Exception:
                pass
        servo.portHandler.closePort()


if __name__ == "__main__":
    raise SystemExit(main())

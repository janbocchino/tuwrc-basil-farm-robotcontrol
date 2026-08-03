# 2. Treiber für drei echte Motoren

Dieses Paket kennt drei ST3215-Servos mit eindeutigen IDs auf einem seriellen
Bus. Es kennt weder Gazebo noch MoveIt-internes Planning.

Zusätzlich enthält es einen passiven Leser für sechs Motoren, der für die
SO-101-Vorbereitung keine URDF benötigt.

## Datenfluss

```text
ROS-Trajektorie mit drei Positionen in rad
        ↓
six_motor_driver
        ↓ Umrechnung: rad → ST3215-Schritte
Servo-IDs 1, 2, 3
        ↓ gemessene Schritte
six_motor_driver
        ↓ Umrechnung: Schritte → rad
/joint_states
```

## Dateien

- `package.xml`: ROS-Abhängigkeiten wie `rclpy` und `control_msgs`.
- `setup.py`: installiert das Python-Paket und erzeugt den ausführbaren
  Befehl `st3215_driver`.
- `setup.cfg`: bestimmt, wohin ROS den ausführbaren Befehl installiert.
- `resource/six_motor_driver`: registriert das Paket im Ament-Index.
- `six_motor_driver/st3215_driver.py`: serielle Kommunikation, Sicherheit,
  Action-Server und Joint-State-Publisher.
- `six_motor_driver/configure_servo_id.py`: weist einem einzeln
  angeschlossenen Servo eine eindeutige Bus-ID zu.
- `six_motor_driver/six_motor_position_reader.py`: liest die Positionen der
  IDs 1 bis 6, ohne Drehmoment oder Bewegungsbefehle zu setzen.
- `six_motor_driver/cyclic_calibration.py`: rechnet zyklische Rohwerte in
  stetige ROS-Winkel um und zurück.
- `config/six_motor_calibration.yaml`: manuelle Grenzen, Nullwerte und
  Richtungen aller sechs Motoren.
- `six_motor_driver/print_moveit_limits.py`: erzeugt daraus URDF- und
  MoveIt-Grenzen.
- `six_motor_driver/calibrate_six_endpoints.py`: erfasst die Endpositionen
  aller sechs Servos und berechnet ihre individuellen Mitten.
- `six_motor_driver/six_motor_driver.py`: bewegt sechs kalibrierte Servos und
  stellt einen `FollowJointTrajectory`-Action-Server bereit.
- `launch/read_six_positions.launch.py`: startet den passiven Lesemodus.

## Wichtige ROS-Schnittstellen

- Eingang:
  `/real_motor_controller/follow_joint_trajectory`
- Ausgang:
  `/joint_states`

`robot_state_publisher` benutzt `/joint_states`, um das digitale Modell auf
die tatsächlich gemessene Motorposition zu setzen.

Beim Start prüft der Treiber zuerst alle IDs und fährt anschließend alle drei
Servos mit `250 Schritte/s` auf die absolute Mittelstellung `2048`. Erst
danach wird diese Lage als `0 rad` verwendet und der Action-Server gestartet.
Mit dem ROS-Parameter `center_on_start:=false` kann die Bewegung für
Wartungszwecke abgeschaltet werden.

## Direkter Start

```bash
ros2 run six_motor_driver st3215_driver --ros-args \
  -p port:=/dev/ttyACM0 \
  -p servo_id_1:=1 \
  -p servo_id_2:=2 \
  -p servo_id_3:=3 \
  -p center_on_start:=true
```

Vor dem gemeinsamen Anschluss müssen die drei Servos unterschiedliche IDs
besitzen. Die genaue Vorgehensweise steht im Haupttutorial.

## Sechs Motoren nur auslesen

Vorher müssen die sechs Servos bei jeweils einzeln angeschlossenem Motor auf
die eindeutigen IDs 1 bis 6 eingestellt werden. Die vollständige Reihenfolge
steht in Abschnitt 2 des Haupttutorials.

Direkt im Terminal anzeigen, ohne ROS-Topics:

```bash
ros2 run six_motor_driver show_six_positions \
  --port /dev/ttyACM0
```

ROS-Leseknoten mit Topics:

```bash
ros2 launch six_motor_driver read_six_positions.launch.py \
  port:=/dev/ttyACM0
```

Topics:

```text
/six_motor/positions_steps
/six_motor/joint_states
```

Ein Grenzbereich darf den Nullübergang enthalten. Die Felder heißen weiterhin
`lower_limits_steps` und `upper_limits_steps`, bedeuten bei der
Sechs-Motor-Kalibrierung aber Start und Ende des erlaubten Bogens:

```yaml
lower_limits_steps: [3000, 0, 0, 0, 0, 0]
upper_limits_steps: [1200, 4095, 4095, 4095, 4095, 4095]
zero_positions_steps: [0, 2048, 2048, 2048, 2048, 2048]
directions: [1, 1, -1, 1, 1, 1]
```

`directions: -1` invertiert die ROS-/Slider-Richtung eines Motors. Aktuell ist
Gelenk 3 invertiert.

Passende lineare Grenzen für URDF und MoveIt:

```bash
ros2 run six_motor_driver print_moveit_limits
```

Endpunkte erfassen:

```bash
ros2 run six_motor_driver calibrate_six_endpoints \
  --port /dev/ttyACM0
```

Das Werkzeug fragt pro Servo nach zwei mechanischen Maximalpositionen und
danach nach einer sicheren Position dazwischen. Damit wird automatisch
entschieden, welcher der zwei möglichen zyklischen Wege erlaubt ist.
Die gespeicherten Grenzen liegen standardmäßig 10 Schritte innerhalb der
mechanischen Maximalpositionen. Zusätzlich wird eine RViz-Nullposition
erfasst, die später `0 rad` im digitalen Modell entspricht.

Einzelnen Servo neu kalibrieren:

```bash
ros2 run six_motor_driver calibrate_six_endpoints \
  --port /dev/ttyACM0 --only 3
```

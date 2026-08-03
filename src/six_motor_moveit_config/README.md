# MoveIt-Konfiguration für den SO-101

Dieses Paket verbindet MoveIt mit dem lokalen SO-101-Modell und dem echten
sechs-Motor-Treiber.

Ausführliches Tutorial:

```text
~/ros2_ws/src/six_motor_system/MOVEIT_TUTORIAL.md
```

## Start

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export PORT=/dev/ttyACM1

ros2 launch six_motor_moveit_config moveit.launch.py \
  port:=$PORT
```

Den Port vorher mit `ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null` prüfen.

## Wichtigste Dateien

- `config/so101.srdf`: MoveIt-Gruppe `so101_arm` und Named States.
- `config/joint_limits.yaml`: MoveIt-Grenzen aus der ST3215-Kalibrierung.
- `config/controllers_real.yaml`: Verbindung zu
  `/six_motor_controller/follow_joint_trajectory`.
- `launch/moveit.launch.py`: startet Robotermodell, optional Hardwaretreiber,
  `move_group` und RViz.

## RViz

Im MotionPlanning-Panel:

1. `Planning Group`: `so101_arm`
2. `Goal State`: `small_test` oder `zero`
3. `Plan`
4. `Execute`

Für echte Hardware am Anfang lieber kleine Bewegungen testen.

// Rail stepper controller (Arduino Uno) for tuwrc_mock_hardware/rail_driver.
// Serial protocol at 115200 baud, one command per line.
//
// Position is counted in steps from the left end (lm_left = 0), increasing
// to the right. It is only meaningful after calibration ("h").
//
// rail_driver relies on:
//   "Target reached, stopped." / "Stopped ..."  - a move ended
//   "Calibrated <length_steps>"                 - calibration succeeded
//   "Calibration failed: <reason>"              - calibration aborted
//   "ST <lm_left> <lm_right> <pos_steps> <homed>" every 20 ms (1 = closed)

int driverPUL = 6;    // PUL- pin
int driverDIR = 7;    // DIR- pin
int lmLeftPin = 5;    // lm_left: switch between D5 and GND (internal pull-up)
int lmRightPin = 4;   // lm_right: switch between D4 and GND (internal pull-up)

// Which DIR level moves the carriage towards lm_left. If calibration reports
// "wrong direction", swap these two.
const boolean DIR_LEFT = LOW;
const boolean DIR_RIGHT = HIGH;

int pd = 50;               // Pulse delay (microseconds) for normal moves
int homingPd = 100;        // Slower pulse delay while calibrating
const long maxTravelSteps = 300000;  // Abort calibration after 3 m of travel
const byte debounceReads = 5;        // Consecutive closed reads to count as closed

enum Mode { IDLE, MOVE, HOME_LEFT, HOME_RIGHT, HOME_CENTER, HOME_ZERO };
Mode mode = IDLE;

boolean setdir = DIR_LEFT;
long targetSteps = 0;  // Steps to take for the current move
long stepsDone = 0;    // Steps completed so far in the current move

long pos = 0;          // Steps from lm_left
boolean homed = false;
long railLength = 0;   // Steps from lm_left to lm_right

byte lmLeftCount = 0;
byte lmRightCount = 0;
boolean lmLeft = false;
boolean lmRight = false;

const unsigned long statusPeriodMs = 20;  // 50 Hz status report
unsigned long lastStatusReport = 0;

void setup() {
  pinMode(driverPUL, OUTPUT);
  pinMode(driverDIR, OUTPUT);
  pinMode(lmLeftPin, INPUT_PULLUP);
  pinMode(lmRightPin, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println("Stepper control ready.");
  Serial.println("Commands:");
  Serial.println("  f<N>     - move N steps towards lm_left, e.g. f2000");
  Serial.println("  r<N>     - move N steps towards lm_right, e.g. r2000");
  Serial.println("  a<N>     - move to absolute step N from lm_left, e.g. a50000");
  Serial.println("  h        - calibrate: lm_left, lm_right, middle, then 0");
  Serial.println("  s        - stop immediately");
  Serial.println("  v<N>     - set speed (pulse delay in microseconds), e.g. v1500");
  Serial.println("  p        - print progress of current move");
  Serial.println("Status every 20 ms: ST <lm_left> <lm_right> <pos_steps> <homed>");
}

void updateSwitches() {
  lmLeftCount = digitalRead(lmLeftPin) == LOW ? min(lmLeftCount + 1, 255) : 0;
  lmRightCount = digitalRead(lmRightPin) == LOW ? min(lmRightCount + 1, 255) : 0;
  lmLeft = lmLeftCount >= debounceReads;
  lmRight = lmRightCount >= debounceReads;
}

void startMove(boolean dir, long n, Mode m) {
  setdir = dir;
  targetSteps = n;
  stepsDone = 0;
  mode = m;
}

void startMoveTo(long target, Mode m) {
  long delta = target - pos;
  startMove(delta >= 0 ? DIR_RIGHT : DIR_LEFT, abs(delta), m);
}

void failCalibration(const char *reason) {
  mode = IDLE;
  homed = false;
  Serial.print("Calibration failed: ");
  Serial.println(reason);
}

// Called when the current move covered all its steps.
void onMoveDone() {
  switch (mode) {
    case MOVE:
      mode = IDLE;
      Serial.println("Target reached, stopped.");
      break;
    case HOME_LEFT:
      failCalibration("lm_left not reached within max travel");
      break;
    case HOME_RIGHT:
      failCalibration("lm_right not reached within max travel");
      break;
    case HOME_CENTER:
      startMoveTo(0, HOME_ZERO);
      break;
    case HOME_ZERO:
      mode = IDLE;
      Serial.print("Calibrated ");
      Serial.println(railLength);
      break;
    default:
      mode = IDLE;
  }
}

// Called when the switch in the direction of travel closed.
void onLimit(boolean leftSide) {
  if (leftSide) {
    pos = 0;  // lm_left defines 0; re-reference on every hit
  } else if (homed) {
    pos = railLength;
  }

  switch (mode) {
    case MOVE:
      mode = IDLE;
      Serial.println(leftSide ? "Stopped at lm_left" : "Stopped at lm_right");
      break;
    case HOME_LEFT:
      startMove(DIR_RIGHT, maxTravelSteps, HOME_RIGHT);
      break;
    case HOME_RIGHT:
      railLength = pos;
      homed = true;
      startMoveTo(railLength / 2, HOME_CENTER);
      break;
    case HOME_CENTER:
      failCalibration("lm_left closed while moving to the middle");
      break;
    case HOME_ZERO:
      // lm_left closed just before the counted 0; pos is now exactly 0.
      mode = IDLE;
      Serial.print("Calibrated ");
      Serial.println(railLength);
      break;
    default:
      mode = IDLE;
      Serial.println("Stopped at limit");
  }
}

void handleCommand(String cmd) {
  char c = cmd.charAt(0);
  String rest = cmd.substring(1);
  rest.trim();

  if (c == 'f' || c == 'F' || c == 'r' || c == 'R') {
    long n = rest.toInt();
    if (n > 0) {
      boolean left = (c == 'f' || c == 'F');
      startMove(left ? DIR_LEFT : DIR_RIGHT, n, MOVE);
      Serial.print(left ? "Moving left " : "Moving right ");
      Serial.print(n);
      Serial.println(" steps");
    } else {
      Serial.println("Specify steps, e.g. f2000");
    }
  } else if (c == 'a' || c == 'A') {
    long target = rest.toInt();
    startMoveTo(target, MOVE);
    Serial.print("Moving to ");
    Serial.println(target);
  } else if (c == 'h' || c == 'H') {
    homed = false;
    startMove(DIR_LEFT, maxTravelSteps, HOME_LEFT);
    Serial.println("Calibrating");
  } else if (c == 's' || c == 'S') {
    boolean wasHoming = (mode == HOME_LEFT || mode == HOME_RIGHT ||
                         mode == HOME_CENTER || mode == HOME_ZERO);
    mode = IDLE;
    if (wasHoming) {
      failCalibration("stopped");
    }
    Serial.println("Stopped");
  } else if (c == 'v' || c == 'V') {
    int val = rest.toInt();
    if (val > 0) {
      pd = val;
      Serial.print("Speed set, pulse delay = ");
      Serial.println(pd);
    } else {
      Serial.println("Specify delay, e.g. v1500");
    }
  } else if (c == 'p' || c == 'P') {
    Serial.print("Step ");
    Serial.print(stepsDone);
    Serial.print(" / ");
    Serial.println(targetSteps);
  } else {
    Serial.println("Unrecognized command");
  }
}

void loop() {
  updateSwitches();

  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      handleCommand(cmd);
    }
  }

  if (mode != IDLE) {
    boolean movingLeft = (setdir == DIR_LEFT);
    if (mode == HOME_LEFT && lmRight) {
      failCalibration("wrong direction: lm_right closed while moving left, swap DIR_LEFT/DIR_RIGHT");
    } else if (movingLeft && lmLeft) {
      onLimit(true);
    } else if (!movingLeft && lmRight) {
      onLimit(false);
    } else if (stepsDone >= targetSteps) {
      onMoveDone();
    } else {
      int stepDelay = (mode == MOVE) ? pd : homingPd;
      digitalWrite(driverDIR, setdir);
      digitalWrite(driverPUL, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(driverPUL, LOW);
      delayMicroseconds(stepDelay);
      stepsDone++;
      pos += movingLeft ? -1 : 1;
    }
  }

  // Non-blocking: ~20 bytes into the TX buffer, 50 Hz x 20 B = 1 kB/s,
  // about 9% of 115200 baud.
  unsigned long now = millis();
  if (now - lastStatusReport >= statusPeriodMs) {
    lastStatusReport = now;
    Serial.print("ST ");
    Serial.print(lmLeft ? 1 : 0);
    Serial.print(" ");
    Serial.print(lmRight ? 1 : 0);
    Serial.print(" ");
    Serial.print(pos);
    Serial.print(" ");
    Serial.println(homed ? 1 : 0);
  }
}

/* Termit Omni hardware stepping, protocol v2.
 * Build: esp32:esp32:esp32 core 3.3.11, FastAccelStepper 1.2.7.
 * F STEP33 DIR32; R STEP23 DIR22; L STEP19 DIR18.
 * F/R shared EN21, L EN17; LOW = powered.
 * s F R L: ramped steps/s; c N: steps/s^2; k: heartbeat; v: version.
 * stop/x: emergency stop; e 0/1: power on/off; a 0/1: hold/auto-sleep.
 * w ms: watchdog; r: zero odom when stationary.
 */
#include <FastAccelStepper.h>
constexpr uint8_t STEP_PINS[] = {33, 23, 19};
constexpr uint8_t DIR_PINS[] = {32, 22, 18};
constexpr uint8_t EN_PINS[] = {21, 17};
constexpr long MAX_SPEED = 8000;
constexpr char VERSION[] = "TERMIT_FASTACCEL_V2";
FastAccelStepperEngine engine;
FastAccelStepper* motors[3] = {};
long targets[3] = {};
char inputBuf[96];
size_t bufIdx = 0;
bool overflowLine = false, driversActive = false, autoSleepEnabled = true;
bool ready = false, timedTest = false, watchdogArmed = false;
uint32_t testEnd = 0, lastCmd = 0, lastMotion = 0, lastOdom = 0;
uint32_t watchdogMs = 500;

bool moving() {
  for (auto* m : motors) if (m && m->isRunning()) return true;
  return false;
}
void power(bool on) {
  if (on == driversActive) return;
  for (auto pin : EN_PINS) digitalWrite(pin, on ? LOW : HIGH);
  driversActive = on;
  if (on) delayMicroseconds(1000);
  lastMotion = millis();
}
void halt(bool emergency) {
  for (int i = 0; i < 3; ++i) {
    targets[i] = 0;
    if (!motors[i]) continue;
    // Emergency abort may lose the last in-flight step in odometry.
    if (emergency) motors[i]->forceStopAndNewPosition(motors[i]->getCurrentPosition());
    else motors[i]->stopMove();
  }
  timedTest = watchdogArmed = false;
  lastMotion = millis();
}
void setTargets(long f, long r, long l) {
  long requested[] = {f, r, l};
  if (f || r || l) power(true);
  for (int i = 0; i < 3; ++i) {
    long value = constrain(requested[i], -MAX_SPEED, MAX_SPEED);
    // A zero target must also cancel an active finite move.
    if (value == targets[i] && (value || !motors[i]->isRunning())) continue;
    targets[i] = value;
    if (!value) motors[i]->stopMove();
    else {
      motors[i]->setSpeedInHz(abs(value));
      // FastAccelStepper decelerates before reversing the direction pin.
      if (value > 0) motors[i]->runForward();
      else motors[i]->runBackward();
    }
  }
  lastCmd = millis();
  watchdogArmed = true;
  timedTest = false;
}
void parseCommand(const char* cmd) {
  char extra;
  long f, r, l, val;
  int idx, dir;
  if (!strcmp(cmd, "v")) { if (ready) Serial.println(VERSION); return; }
  if (!strcmp(cmd, "stop") || !strcmp(cmd, "x")) { halt(true); return; }
  if (!ready) return;
  if (!strcmp(cmd, "q")) {
    Serial.printf("q %d %d %lu\n", driversActive, moving(), (unsigned long)watchdogMs);
    return;
  }
  if (!strcmp(cmd, "k")) { lastCmd = millis(); return; }
  if (sscanf(cmd, "s %ld %ld %ld %c", &f, &r, &l, &extra) == 3) {
    setTargets(f, r, l);
  } else if (sscanf(cmd, "c %ld %c", &val, &extra) == 1) {
    if (val >= 100 && val <= 20000)
      for (auto* m : motors) { m->setAcceleration(val); m->applySpeedAcceleration(); }
  } else if (sscanf(cmd, "e %ld %c", &val, &extra) == 1 && (val == 0 || val == 1)) {
    if (val == 1) halt(true);
    power(val == 0);
  } else if (sscanf(cmd, "a %ld %c", &val, &extra) == 1 && (val == 0 || val == 1)) {
    autoSleepEnabled = (val == 1);
    if (!autoSleepEnabled) power(true);
  } else if (sscanf(cmd, "w %ld %c", &val, &extra) == 1) {
    if (val >= 100 && val <= 10000) watchdogMs = val;
  } else if (!strcmp(cmd, "r")) {
    if (!moving()) for (auto* m : motors) m->setCurrentPosition(0);
  } else if (sscanf(cmd, "t %d %ld %ld %c", &idx, &val, &f, &extra) == 3) {
    if (idx < 0 || idx > 2 || f < 1 || f > 10000) return;
    halt(true);
    long speeds[] = {0, 0, 0};
    speeds[idx] = constrain(val, -MAX_SPEED, MAX_SPEED);
    setTargets(speeds[0], speeds[1], speeds[2]);
    timedTest = true;
    testEnd = millis() + f;
  } else if (sscanf(cmd, "m %d %ld %c", &idx, &val, &extra) == 2) {
    if (idx < 0 || idx > 2 || val < -80000 || val > 80000 || moving()) return;
    power(true);
    motors[idx]->setSpeedInHz(1000);
    motors[idx]->move(val);
    lastCmd = millis();
    watchdogArmed = true;
  } else if (sscanf(cmd, "h %d %d %ld %c", &idx, &dir, &val, &extra) == 3) {
    if (idx < 0 || idx > 2 || (dir != 0 && dir != 1) || val < 0 || val > 80000 || moving()) return;
    power(true);
    motors[idx]->setSpeedInHz(1000);
    motors[idx]->move(dir ? val : -val);
    lastCmd = millis();
    watchdogArmed = true;
  }
}
void setup() {
  for (auto pin : EN_PINS) { digitalWrite(pin, HIGH); pinMode(pin, OUTPUT); }
  pinMode(2, OUTPUT);
  Serial.begin(115200);
  engine.init();
  ready = true;
  for (int i = 0; i < 3; ++i) {
    motors[i] = engine.stepperConnectToPin(STEP_PINS[i]);
    if (!motors[i]) { ready = false; break; }
    motors[i]->setDirectionPin(DIR_PINS[i], true, 50);
    motors[i]->setAutoEnable(false); // Shared enables managed together above.
    motors[i]->setAcceleration(1600);
    motors[i]->setSpeedInHz(1000);
  }
  lastCmd = lastMotion = millis();
  Serial.println(ready ? VERSION : "ERROR_STEPPER_INIT");
}
void loop() {
  // Bound UART work per iteration; hardware stepping is independent of loop.
  for (int n = 0; n < 96 && Serial.available(); ++n) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufIdx && !overflowLine) { inputBuf[bufIdx] = 0; parseCommand(inputBuf); }
      bufIdx = 0;
      overflowLine = false;
    } else if (bufIdx < sizeof(inputBuf) - 1) inputBuf[bufIdx++] = c;
    else overflowLine = true;
  }
  uint32_t now = millis();
  if (!ready) { delay(1); return; }
  if (timedTest && int32_t(now - testEnd) >= 0) halt(false);
  if (!timedTest && watchdogArmed && uint32_t(now - lastCmd) > watchdogMs) halt(false);
  if (moving()) lastMotion = now;
  if (autoSleepEnabled && !moving() && uint32_t(now - lastMotion) >= 2000) power(false);
  digitalWrite(2, (now / 250) % 2);
  if (uint32_t(now - lastOdom) >= 50) {
    lastOdom = now;
    char line[96];
    int len = snprintf(line, sizeof(line), "o %ld %ld %ld %ld %ld %ld\n",
      (long)motors[0]->getCurrentPosition(), (long)motors[1]->getCurrentPosition(),
      (long)motors[2]->getCurrentPosition(), (long)(motors[0]->getCurrentSpeedInMilliHz() / 1000),
      (long)(motors[1]->getCurrentSpeedInMilliHz() / 1000), (long)(motors[2]->getCurrentSpeedInMilliHz() / 1000));
    if (len > 0 && len < sizeof(line) && Serial.availableForWrite() >= len)
      Serial.write((uint8_t*)line, len);
  }
  delay(1);
}

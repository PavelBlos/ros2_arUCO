/*
  ESP32 Stepper Driver - 3-Wheel Omni Robot with AUTO-SLEEP (Zero Hissing)
  ========================================================================
  Подключение всех 3 моторов:
  
  Мотор 1 (Передний F):
    - CLK+ (STEP) -> D33
    - CW+  (DIR)  -> D32
    - EN+  (ENA)  -> D25  (ВАЖНО: пин 25 прямо рядом с 33, полноценный выход!)

  Мотор 2 (Правый R):
    - CLK+ (STEP) -> D23
    - CW+  (DIR)  -> D22
    - EN+  (ENA)  -> D21

  Мотор 3 (Левый L):
    - CLK+ (STEP) -> D19
    - CW+  (DIR)  -> D18 (спаян с D5)
    - EN+  (ENA)  -> D17

  Светодиод статуса: GPIO 2
*/

#include <AccelStepper.h>

// Мотор 1 (F)
#define M1_STEP_PIN 33
#define M1_DIR_PIN  32
#define M1_EN_PIN   21  // Соединен вместе с Мотором 2 на пин 21 (так как пин 34 только вход)

// Мотор 2 (R)
#define M2_STEP_PIN 23
#define M2_DIR_PIN  22
#define M2_EN_PIN   21

// Мотор 3 (L)
#define M3_STEP_PIN 19
#define M3_DIR_PIN  18
#define M3_EN_PIN   17

#define LED_PIN 2

#define MAX_SPEED_LIMIT 8000.0
#define MIN_PULSE_WIDTH 4 // 4 мкс для быстрого и плавного шагания

AccelStepper stepperF(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper stepperR(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);
AccelStepper stepperL(AccelStepper::DRIVER, M3_STEP_PIN, M3_DIR_PIN);

const int MAX_BUF = 64;
char inputBuf[MAX_BUF];
int bufIdx = 0;

unsigned long lastCmdTime = 0;
unsigned long watchdogTimeoutMs = 500; // Настраиваемый Watchdog (по умолчанию 500 мс)
bool motorsStopped = true;

unsigned long testEndTime = 0;
int activeTestMotor = -1;

unsigned long lastBlinkTime = 0;
bool ledState = false;

// Одометрия: периодическая отправка шагов
unsigned long lastOdomTime = 0;
const unsigned long ODOM_INTERVAL_MS = 50; // 20 Гц (каждые 50 мс)

// =========================================================================
// АВТОМАТИЧЕСКИЙ РЕЖИМ СНА (ПОЛНАЯ ТИШИНА В ПРОСТОЕ)
// =========================================================================
bool driversActive = false;
bool autoSleepEnabled = true; // true = авто-сон (тишина), false = постоянное удержание (жесткий вал)
unsigned long lastMotionTime = 0;
const unsigned long AUTO_SLEEP_DELAY_MS = 1500; // Через 1.5 сек после остановки -> выключаем ток

void enableDrivers() {
  if (!driversActive) {
    // LOW включает питание обмоток (для драйверов с общим катодом)
    digitalWrite(M1_EN_PIN, LOW);
    digitalWrite(M2_EN_PIN, LOW);
    digitalWrite(M3_EN_PIN, LOW);
    driversActive = true;
    delayMicroseconds(50); // Мгновенное включение ключей
  }
}

void sleepDrivers() {
  if (driversActive) {
    // HIGH отключает питание обмоток (тишина 0 дБ, моторы не шипят и холодные)
    digitalWrite(M1_EN_PIN, HIGH);
    digitalWrite(M2_EN_PIN, HIGH);
    digitalWrite(M3_EN_PIN, HIGH);
    driversActive = false;
  }
}

void stopMotors() {
  stepperF.setSpeed(0);
  stepperR.setSpeed(0);
  stepperL.setSpeed(0);
  motorsStopped = true;
  activeTestMotor = -1;
  lastMotionTime = millis();
}

// Прямой аппаратный тест генерации шагов
void hardwareStep(int motorIdx, int dir, int steps, int speedDelayUs) {
  enableDrivers();
  int stepPin = M1_STEP_PIN;
  int dirPin = M1_DIR_PIN;
  if (motorIdx == 1) { stepPin = M2_STEP_PIN; dirPin = M2_DIR_PIN; }
  else if (motorIdx == 2) { stepPin = M3_STEP_PIN; dirPin = M3_DIR_PIN; }

  digitalWrite(dirPin, dir ? HIGH : LOW);
  delayMicroseconds(50);
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speedDelayUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speedDelayUs);
  }
  lastMotionTime = millis();
}

void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Настройка пинов Enable
  pinMode(M1_EN_PIN, OUTPUT);
  pinMode(M2_EN_PIN, OUTPUT);
  pinMode(M3_EN_PIN, OUTPUT);

  // Стартуем сразу в режиме ТИШИНЫ (сон)
  driversActive = true;
  sleepDrivers();

  // Настройка пинов шаговиков
  pinMode(M1_STEP_PIN, OUTPUT);
  pinMode(M1_DIR_PIN,  OUTPUT);
  pinMode(M2_STEP_PIN, OUTPUT);
  pinMode(M2_DIR_PIN,  OUTPUT);
  pinMode(M3_STEP_PIN, OUTPUT);
  pinMode(M3_DIR_PIN,  OUTPUT);

  digitalWrite(M1_STEP_PIN, LOW);
  digitalWrite(M1_DIR_PIN,  LOW);
  digitalWrite(M2_STEP_PIN, LOW);
  digitalWrite(M2_DIR_PIN,  LOW);
  digitalWrite(M3_STEP_PIN, LOW);
  digitalWrite(M3_DIR_PIN,  LOW);

  // Настройка AccelStepper
  stepperF.setMinPulseWidth(MIN_PULSE_WIDTH);
  stepperR.setMinPulseWidth(MIN_PULSE_WIDTH);
  stepperL.setMinPulseWidth(MIN_PULSE_WIDTH);

  stepperF.setMaxSpeed(MAX_SPEED_LIMIT);
  stepperR.setMaxSpeed(MAX_SPEED_LIMIT);
  stepperL.setMaxSpeed(MAX_SPEED_LIMIT);

  stopMotors();

  Serial.println("TERMIT_SILENT_STEPPER_READY");
  lastCmdTime = millis();
  lastMotionTime = millis();
}

void parseCommand(char* cmd) {
  // 1. Установка скоростей: "s <speed_F> <speed_R> <speed_L>"
  if (cmd[0] == 's') {
    long speedF = 0, speedR = 0, speedL = 0;
    int parsed = sscanf(cmd, "s %ld %ld %ld", &speedF, &speedR, &speedL);
    if (parsed == 3) {
      if (speedF != 0 || speedR != 0 || speedL != 0) {
        enableDrivers();
        lastMotionTime = millis();
        motorsStopped = false;
      } else {
        motorsStopped = true;
      }
      
      speedF = constrain(speedF, -MAX_SPEED_LIMIT, MAX_SPEED_LIMIT);
      speedR = constrain(speedR, -MAX_SPEED_LIMIT, MAX_SPEED_LIMIT);
      speedL = constrain(speedL, -MAX_SPEED_LIMIT, MAX_SPEED_LIMIT);
      
      stepperF.setSpeed(speedF);
      stepperR.setSpeed(speedR);
      stepperL.setSpeed(speedL);
      
      lastCmdTime = millis();
      activeTestMotor = -1;
    }
  }
  // 2. Тест отдельного мотора: "t <motor_idx 0..2> <speed> <duration_ms>"
  else if (cmd[0] == 't') {
    int mIdx = 0, spd = 2000, dur = 2000;
    int parsed = sscanf(cmd, "t %d %d %d", &mIdx, &spd, &dur);
    if (parsed >= 1) {
      enableDrivers();
      stopMotors();
      long stepSpd = (abs(spd) < 500) ? (spd * 30) : spd;
      if (stepSpd == 0) stepSpd = 1500;
      stepSpd = constrain(stepSpd, -MAX_SPEED_LIMIT, MAX_SPEED_LIMIT);

      if (mIdx == 0) stepperF.setSpeed(stepSpd);
      else if (mIdx == 1) stepperR.setSpeed(stepSpd);
      else if (mIdx == 2) stepperL.setSpeed(stepSpd);

      activeTestMotor = mIdx;
      testEndTime = millis() + (dur > 0 ? dur : 2000);
      lastCmdTime = millis();
      lastMotionTime = millis();
      motorsStopped = false;
    }
  }
  // 3. Микротест мотора на точное число шагов: "m <motor_idx 0..2> <steps>"
  else if (cmd[0] == 'm') {
    int mIdx = 0, steps = 200;
    int parsed = sscanf(cmd, "m %d %d", &mIdx, &steps);
    if (parsed >= 2) {
      enableDrivers();
      stopMotors();
      int dir = (steps >= 0) ? 1 : 0;
      hardwareStep(mIdx, dir, abs(steps), 400);
      lastMotionTime = millis();
    }
  }
  // 4. Прямой аппаратный тест: "h <motor_idx 0..2> <dir 0/1> <steps>"
  else if (cmd[0] == 'h') {
    int mIdx = 0, dir = 1, steps = 1600;
    sscanf(cmd, "h %d %d %d", &mIdx, &dir, &steps);
    hardwareStep(mIdx, dir, steps, 300);
  }
  // 5. Ручное переключение уровня Enable: "e 0" (включить ток) или "e 1" (выключить ток)
  else if (cmd[0] == 'e') {
    int lvl = 0;
    sscanf(cmd, "e %d", &lvl);
    if (lvl == 0) enableDrivers();
    else sleepDrivers();
  }
  // 6. Настройка режима удержания: "a 1" (авто-сон) или "a 0" (постоянное удержание)
  else if (cmd[0] == 'a') {
    int val = 1;
    sscanf(cmd, "a %d", &val);
    autoSleepEnabled = (val != 0);
    if (!autoSleepEnabled) {
      enableDrivers(); // При отключении авто-сна сразу включаем удержание
    }
  }
  // 7. Настройка таймаута Watchdog: "w <timeout_ms>" (например "w 500")
  else if (cmd[0] == 'w') {
    unsigned long val = 500;
    sscanf(cmd, "w %lu", &val);
    if (val >= 100 && val <= 10000) {
      watchdogTimeoutMs = val;
    }
  }
  // 8. Сброс одометрии (счетчиков шагов в 0): "r"
  else if (cmd[0] == 'r') {
    stepperF.setCurrentPosition(0);
    stepperR.setCurrentPosition(0);
    stepperL.setCurrentPosition(0);
  }
  // 9. Стоп
  else if (strcmp(cmd, "stop") == 0 || cmd[0] == 'x') {
    stopMotors();
  }
}

void loop() {
  unsigned long now = millis();

  // 1. Генерация шагов для всех трех моторов
  stepperF.runSpeed();
  stepperR.runSpeed();
  stepperL.runSpeed();

  // 2. Heartbeat светодиод
  if (now - lastBlinkTime >= 250) {
    lastBlinkTime = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }

  // 3. Отправка одометрии и скоростей моторов (20 Гц)
  if (now - lastOdomTime >= ODOM_INTERVAL_MS) {
    lastOdomTime = now;
    Serial.printf("o %ld %ld %ld %ld %ld %ld\n",
      stepperF.currentPosition(),
      stepperR.currentPosition(),
      stepperL.currentPosition(),
      (long)stepperF.speed(),
      (long)stepperR.speed(),
      (long)stepperL.speed()
    );
  }

  // 4. Чтение команд по Serial
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufIdx > 0) {
        inputBuf[bufIdx] = '\0';
        parseCommand(inputBuf);
        bufIdx = 0;
      }
    } else if (bufIdx < MAX_BUF - 1) {
      inputBuf[bufIdx++] = c;
    }
  }

  // 5. Окончание теста отдельного мотора
  if (activeTestMotor >= 0 && now >= testEndTime) {
    stopMotors();
  }

  // 6. Watchdog безопасности (остановка скорости при потере связи)
  if (!motorsStopped && activeTestMotor < 0 && (now - lastCmdTime > watchdogTimeoutMs)) {
    stopMotors();
  }

  // 7. АВТО-СОН: если включен авто-сон и моторы остановлены дольше AUTO_SLEEP_DELAY_MS -> гасим ток
  if (autoSleepEnabled && motorsStopped && activeTestMotor < 0 && driversActive && (now - lastMotionTime > AUTO_SLEEP_DELAY_MS)) {
    sleepDrivers();
  }
}

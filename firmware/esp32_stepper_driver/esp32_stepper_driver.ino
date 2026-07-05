/*
  ESP32 Stepper Driver for Termit Omni Wheel Robot
  ================================================
  Управление 3 шаговыми двигателями NEMA через драйверы TB6600/TB6560.
  
  Протокол обмена по Serial (115200 baud):
  - Прием (Малина -> ESP32): "s <speed_F> <speed_R> <speed_L>\n" (скорость в шагах/сек)
  - Отправка (ESP32 -> Малина): "o <pos_F> <pos_R> <pos_L>\n" (текущее положение в шагах)
*/

#include <AccelStepper.h>

// =========================================================================
// НАСТРОЙКА ПИНОВ (Измените эти значения под ваше физическое подключение)
// =========================================================================

// Передний мотор (Forward/Front)
#define F_STEP_PIN 12
#define F_DIR_PIN  13

// Правый мотор (Right)
#define R_STEP_PIN 14
#define R_DIR_PIN  15

// Левый мотор (Left)
#define L_STEP_PIN 16
#define L_DIR_PIN  17

// Общий пин включения драйверов (Enable). 
// Если ваши драйверы включены постоянно (выводы EN висят в воздухе), оставьте -1.
#define COMMON_ENABLE_PIN -1 

// =========================================================================
// НАСТРОЙКИ СКОРОСТЕЙ И УСКОРЕНИЙ ПО УМОЛЧАНИЮ
// =========================================================================
#define MAX_SPEED_LIMIT 5000.0 // Максимальная частота шагов (шагов/сек)

// Инициализация моторов в режиме драйвера (step/dir)
AccelStepper stepperF(AccelStepper::DRIVER, F_STEP_PIN, F_DIR_PIN);
AccelStepper stepperR(AccelStepper::DRIVER, R_STEP_PIN, R_DIR_PIN);
AccelStepper stepperL(AccelStepper::DRIVER, L_STEP_PIN, L_DIR_PIN);

// Переменные для работы последовательного порта
const int MAX_BUF = 64;
char inputBuf[MAX_BUF];
int bufIdx = 0;

// Таймеры для отправки одометрии и защиты (Watchdog)
unsigned long lastOdomTime = 0;
const unsigned long ODOM_INTERVAL = 40; // Интервал отправки одометрии (40мс = 25 Гц)

unsigned long lastCmdTime = 0;
const unsigned long WATCHDOG_TIMEOUT = 500; // Таймаут остановки моторов (500мс)
bool motorsStopped = true;

// Уровень сигнала для включения моторов (зависит от схемы подключения EN+ / EN-)
#define ENABLE_LEVEL LOW 

void setup() {
  Serial.begin(115200);
  
  // Настройка пина Enable, если он используется
  if (COMMON_ENABLE_PIN != -1) {
    pinMode(COMMON_ENABLE_PIN, OUTPUT);
    digitalWrite(COMMON_ENABLE_PIN, ENABLE_LEVEL); // Включаем моторы
  }

  // Настройка максимальных скоростей для библиотечных расчетов
  stepperF.setMaxSpeed(MAX_SPEED_LIMIT);
  stepperR.setMaxSpeed(MAX_SPEED_LIMIT);
  stepperL.setMaxSpeed(MAX_SPEED_LIMIT);

  // Сбрасываем начальные скорости в 0
  stepperF.setSpeed(0);
  stepperR.setSpeed(0);
  stepperL.setSpeed(0);

  lastCmdTime = millis();
}

void loop() {
  // 1. Постоянная генерация шагов (должна вызываться как можно чаще)
  stepperF.runSpeed();
  stepperR.runSpeed();
  stepperL.runSpeed();

  // 2. Обработка входящих команд по Serial
  handleSerial();

  // 3. Периодическая отправка одометрии (текущих шагов)
  unsigned long now = millis();
  if (now - lastOdomTime >= ODOM_INTERVAL) {
    lastOdomTime = now;
    sendOdometry();
  }

  // 4. Защита (Watchdog): останавливаемся, если Малина молчит дольше WATCHDOG_TIMEOUT
  checkWatchdog(now);
}

// Функция неблокирующего чтения из Serial
void handleSerial() {
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
}

// Парсер входящих команд
void parseCommand(char* cmd) {
  // Ожидаем команду формата "s <speed_F> <speed_R> <speed_L>"
  if (cmd[0] == 's') {
    long speedF = 0, speedR = 0, speedL = 0;
    int parsed = sscanf(cmd, "s %ld %ld %ld", &speedF, &speedR, &speedL);
    
    if (parsed == 3) {
      // Ограничиваем скорость безопасными лимитами
      speedF = constrain(speedF, -MAX_SPEED_LIMIT, MAX_SPEED_LIMIT);
      speedR = constrain(speedR, -MAX_SPEED_LIMIT, MAX_SPEED_LIMIT);
      speedL = constrain(speedL, -MAX_SPEED_LIMIT, MAX_SPEED_LIMIT);
      
      stepperF.setSpeed(speedF);
      stepperR.setSpeed(speedR);
      stepperL.setSpeed(speedL);
      
      lastCmdTime = millis();
      motorsStopped = false;
    }
  }
}

// Отправка одометрии в формате "o <step_F> <step_R> <step_L>"
void sendOdometry() {
  long posF = stepperF.currentPosition();
  long posR = stepperR.currentPosition();
  long posL = stepperL.currentPosition();
  
  Serial.print("o ");
  Serial.print(posF);
  Serial.print(" ");
  Serial.print(posR);
  Serial.print(" ");
  Serial.println(posL);
}

// Защита от потери связи с управляющим компьютером
void checkWatchdog(unsigned long now) {
  if (!motorsStopped && (now - lastCmdTime > WATCHDOG_TIMEOUT)) {
    stepperF.setSpeed(0);
    stepperR.setSpeed(0);
    stepperL.setSpeed(0);
    motorsStopped = true;
    
    // Выводим в лог предупреждение о срабатывании защиты
    Serial.println("w Watchdog triggered - motors stopped");
  }
}

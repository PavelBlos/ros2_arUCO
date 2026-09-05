"""
Termit Omni Robot - Industrial UART/Serial Python API
=====================================================
Высокопроизводительный модуль управления 3-колесным шаговым Omni-роботом "Термит"
для Raspberry Pi, бортовых компьютеров и программ автономной навигации (ROS 2).

Ключевые возможности:
- Управление телом робота (vx, vy в м/с, w в рад/с) и отдельными моторами (шаги/сек)
- Высокоточная одометрия (Dead Reckoning) в реальном времени (20-50 Гц)
- Двусторонний аппаратный Watchdog безопасности (авто-стоп при потере связи)
- Управление энергопотреблением и тишиной (Auto-Sleep / Continuous Hold / Disabled)
- Потокобезопасная архитектура с фоновым разбором телеметрии
"""

import math
import time
import glob
import threading
from enum import Enum
from dataclasses import dataclass
from typing import Tuple, Optional, Callable, List

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


class HoldMode(Enum):
    AUTO_SLEEP = 1       # Авто-сон (тишина 0 дБ через 1.5 сек после остановки)
    CONTINUOUS_HOLD = 2  # Постоянное жесткое удержание вала (ток не снимается)
    DISABLED = 3         # Принудительное полное отключение токов


@dataclass
class RobotConfig:
    """Физические и кинематические параметры робота Termit Omni."""
    wheel_radius: float = 0.030          # Радиус колеса (м) = 30 мм (диаметр 60 мм)
    base_radius: float = 0.122           # Радиус базы робота от центра до колеса (м) = 122 мм
    steps_per_rev: int = 1600            # Микрошагов на 1 полный оборот вала (1/8 шага)
    gear_ratio: float = 1.0              # Передаточное число редуктора (1.0 для прямого привода)
    
    # Ограничения безопасности
    max_linear_speed: float = 0.8        # Максимальная скорость движения (м/с)
    max_angular_speed: float = 4.0       # Максимальная угловая скорость (рад/с)
    max_motor_speed_steps: int = 7000    # Предельная частота шагов драйвера (шагов/сек)
    watchdog_timeout_ms: int = 500       # Аппаратный таймаут watchdog на ESP32 (мс)
    
    # Инверсии моторов (если перепутана фазировка)
    inv_m1: int = 1                      # Мотор 1 (F, передний)
    inv_m2: int = 1                      # Мотор 2 (R, правый)
    inv_m3: int = 1                      # Мотор 3 (L, левый)
    side_ratio: float = 0.50             # Калибровочный коэффициент бокового хода (0.5 для 120 град)


@dataclass
class OdometryData:
    """Данные одометрии и кинематического состояния робота."""
    x: float = 0.0                       # Позиция X в мировой системе координат (м)
    y: float = 0.0                       # Позиция Y в мировой системе координат (м)
    theta: float = 0.0                   # Ориентация робота (рад) [-pi, pi]
    
    vx: float = 0.0                      # Линейная скорость Vx (м/с) в системе робота
    vy: float = 0.0                      # Линейная скорость Vy (м/с) в системе робота
    omega: float = 0.0                   # Угловая скорость (рад/с)
    
    wheel_steps: Tuple[int, int, int] = (0, 0, 0)   # Текущие абсолютные шаги (M1, M2, M3)
    wheel_speeds: Tuple[int, int, int] = (0, 0, 0)  # Текущие скорости моторов (шаги/сек)
    timestamp: float = 0.0               # Временная метка измерения (сек)


class TermitRobotAPI:
    """
    Основной класс API для управления роботом через Serial/UART порт.
    """

    def __init__(self, config: Optional[RobotConfig] = None):
        if not HAS_SERIAL:
            raise RuntimeError("Библиотека pyserial не установлена. Выполните: pip install pyserial")

        self.config = config or RobotConfig()
        
        # Перевод: шагов на 1 метр пути колеса
        self._wheel_circumference = 2.0 * math.pi * self.config.wheel_radius
        self._steps_per_meter = (self.config.steps_per_rev * self.config.gear_ratio) / self._wheel_circumference
        self._meters_per_step = 1.0 / self._steps_per_meter

        # Состояние подключения
        self._ser: Optional[serial.Serial] = None
        self._ser_lock = threading.Lock()
        self._is_connected = False
        self._port_name = ""

        # Потоки чтения телеметрии и Watchdog Heartbeat
        self._stop_event = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None
        self._heartbeat_thread: Optional[threading.Thread] = None

        # Одометрия
        self._odom_lock = threading.Lock()
        self._odom = OdometryData()
        self._last_wheel_steps: Optional[Tuple[int, int, int]] = None
        self._last_odom_time: float = 0.0
        self._odom_callbacks: List[Callable[[OdometryData], None]] = []

        # Целевые скорости для heartbeat
        self._target_cmd: str = "stop"
        self._is_moving = False
        self._last_telemetry_time = 0.0

    # =========================================================================
    # Подключение и управление соединением
    # =========================================================================

    @staticmethod
    def list_available_ports() -> List[str]:
        """Возвращает список доступных Serial-портов в системе."""
        if not HAS_SERIAL:
            return []
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports:
            ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        return ports

    def connect(self, port: Optional[str] = None, baudrate: int = 115200, timeout: float = 0.1) -> bool:
        """
        Подключается к контроллеру ESP32 по Serial/UART.
        Если port не указан, пытается найти автоматически.
        """
        if self._is_connected:
            self.disconnect()

        if port is None:
            available = self.list_available_ports()
            if not available:
                raise ConnectionError("Не найдено ни одного доступного последовательного порта!")
            port = available[0]

        try:
            self._ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                write_timeout=0.2
            )
            
            # Аппаратный сброс DTR/RTS для чистого рестарта ESP32
            self._ser.dtr = False
            self._ser.rts = False
            time.sleep(0.05)
            self._ser.rts = True
            time.sleep(0.05)
            self._ser.rts = False
            time.sleep(0.3)  # Ожидание загрузки загрузчика ESP32

            self._port_name = port
            self._is_connected = True
            self._stop_event.clear()

            # Запуск потока разбора телеметрии
            self._reader_thread = threading.Thread(target=self._telemetry_reader_loop, daemon=True, name="TermitTelemetry")
            self._reader_thread.start()

            # Запуск потока поддержания связи (Heartbeat для ESP32 Watchdog)
            self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True, name="TermitHeartbeat")
            self._heartbeat_thread.start()

            # Настройка таймаута Watchdog на контроллере
            self.set_watchdog_timeout(self.config.watchdog_timeout_ms)
            
            # Режим удержания по умолчанию (Авто-сон для тишины)
            self.set_holding_mode(HoldMode.AUTO_SLEEP)

            # Сброс одометрии
            self.reset_odometry()

            return True

        except Exception as e:
            self.disconnect()
            raise ConnectionError(f"Ошибка открытия порта {port}: {e}")

    def disconnect(self):
        """Корректно останавливает робота и закрывает порт."""
        if self._is_connected:
            try:
                self.stop()
                time.sleep(0.05)
            except Exception:
                pass

        self._is_connected = False
        self._stop_event.set()

        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=0.5)

        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=0.5)

        with self._ser_lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.close()
                except Exception:
                    pass
            self._ser = None

    @property
    def is_connected(self) -> bool:
        """Проверяет статус подключения к контроллеру."""
        return self._is_connected and (time.time() - self._last_telemetry_time < 2.0)

    # =========================================================================
    # Кинематика и управление движением
    # =========================================================================

    def drive(self, vx: float, vy: float, omega: float = 0.0):
        """
        ВЫСОКИЙ УРОВЕНЬ: Управление вектором движения тела робота (Twist).
        
        :param vx: Линейная скорость вдоль оси робота (вправо > 0, влево < 0) в м/с
        :param vy: Линейная скорость вперед/назад (вперед > 0, назад < 0) в м/с
        :param omega: Угловая скорость вращения робота (против часовой > 0) в рад/с
        """
        # Ограничение допустимых пределов
        vx = max(min(vx, self.config.max_linear_speed), -self.config.max_linear_speed)
        vy = max(min(vy, self.config.max_linear_speed), -self.config.max_linear_speed)
        omega = max(min(omega, self.config.max_angular_speed), -self.config.max_angular_speed)

        # Обратная кинематика 3-колесного Omni (Termit):
        # M1 (переднее, ось Y): катится по X -> v_1 = vx + omega * L
        # M2 (правое, -30 град): v_2 = -side_ratio * vx - 0.866 * vy + omega * L
        # M3 (левое, +210 град): v_3 = -side_ratio * vx + 0.866 * vy + omega * L
        L = self.config.base_radius
        v1_mps = (vx + omega * L) * self.config.inv_m1
        v2_mps = (-self.config.side_ratio * vx - 0.866025 * vy + omega * L) * self.config.inv_m2
        v3_mps = (-self.config.side_ratio * vx + 0.866025 * vy + omega * L) * self.config.inv_m3

        # Перевод в шаги/сек
        s1 = int(round(v1_mps * self._steps_per_meter))
        s2 = int(round(v2_mps * self._steps_per_meter))
        s3 = int(round(v3_mps * self._steps_per_meter))

        self.set_motor_speeds(s1, s2, s3)

    def set_motor_speeds(self, m1_steps_s: int, m2_steps_s: int, m3_steps_s: int):
        """
        НИЗКИЙ УРОВЕНЬ: Раздельное управление скоростью каждого мотора по отдельности.
        
        :param m1_steps_s: Скорость Мотора 1 (Передний) в шагах/сек
        :param m2_steps_s: Скорость Мотора 2 (Правый) в шагах/сек
        :param m3_steps_s: Скорость Мотора 3 (Левый) в шагах/сек
        """
        # Ограничение максимальной частоты драйверов
        limit = self.config.max_motor_speed_steps
        s1 = max(min(m1_steps_s, limit), -limit)
        s2 = max(min(m2_steps_s, limit), -limit)
        s3 = max(min(m3_steps_s, limit), -limit)

        if s1 == 0 and s2 == 0 and s3 == 0:
            self._target_cmd = "stop"
            self._is_moving = False
            self._send_raw("stop")
        else:
            cmd = f"s {s1} {s2} {s3}"
            self._target_cmd = cmd
            self._is_moving = True
            self._send_raw(cmd)

    def stop(self):
        """Плавная остановка робота."""
        self._target_cmd = "stop"
        self._is_moving = False
        self._send_raw("stop")

    def emergency_stop(self):
        """Аварийная мгновенная остановка робота (E-STOP)."""
        self._target_cmd = "stop"
        self._is_moving = False
        self._send_raw("stop")
        self._send_raw("stop")

    def microstep(self, motor_idx: int, steps: int):
        """Поворот заданного мотора (0, 1 или 2) ровно на заданное число шагов."""
        self._send_raw(f"m {motor_idx} {steps}")

    # =========================================================================
    # Режимы удержания и Watchdog
    # =========================================================================

    def set_holding_mode(self, mode: HoldMode):
        """
        Управляет удержанием моторов и тишиной в покое.
        - HoldMode.AUTO_SLEEP: авто-сон (тишина 0 дБ через 1.5 сек после остановки)
        - HoldMode.CONTINUOUS_HOLD: постоянная блокировка валов
        - HoldMode.DISABLED: принудительное отключение питания обмоток
        """
        if mode == HoldMode.AUTO_SLEEP:
            self._send_raw("a 1")
        elif mode == HoldMode.CONTINUOUS_HOLD:
            self._send_raw("a 0")
        elif mode == HoldMode.DISABLED:
            self._send_raw("e 1")

    def set_watchdog_timeout(self, timeout_ms: int):
        """Устанавливает аппаратный таймаут безопасности Watchdog на ESP32 (мс)."""
        timeout_ms = max(100, min(timeout_ms, 5000))
        self.config.watchdog_timeout_ms = timeout_ms
        self._send_raw(f"w {timeout_ms}")

    # =========================================================================
    # Одометрия
    # =========================================================================

    def get_odometry(self) -> OdometryData:
        """Возвращает актуальный снимок одометрии робота (потокобезопасно)."""
        with self._odom_lock:
            return OdometryData(
                x=self._odom.x,
                y=self._odom.y,
                theta=self._odom.theta,
                vx=self._odom.vx,
                vy=self._odom.vy,
                omega=self._odom.omega,
                wheel_steps=self._odom.wheel_steps,
                wheel_speeds=self._odom.wheel_speeds,
                timestamp=self._odom.timestamp
            )

    def reset_odometry(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Сбрасывает мировую позу робота и аппаратные счетчики шагов на ESP32."""
        with self._odom_lock:
            self._odom.x = x
            self._odom.y = y
            self._odom.theta = theta
            self._odom.vx = 0.0
            self._odom.vy = 0.0
            self._odom.omega = 0.0
            self._last_wheel_steps = None
        self._send_raw("r")

    def add_odometry_callback(self, callback: Callable[[OdometryData], None]):
        """Регистрирует функцию обратного вызова, вызываемую при каждом пакете одометрии (20 Гц)."""
        self._odom_callbacks.append(callback)

    # =========================================================================
    # Высокоуровневые маневры (проезд на расстояние / поворот на угол)
    # =========================================================================

    def move_distance(self, distance_m: float, heading_rad: float = 0.0, speed_mps: float = 0.2, timeout: float = 15.0) -> bool:
        """
        Автономный маневр: проехать заданное расстояние (м) под заданным углом heading_rad.
        Блокирует выполнение до завершения маневра по данным одометрии.
        """
        start_odom = self.get_odometry()
        start_x, start_y = start_odom.x, start_odom.y
        
        vx = speed_mps * math.cos(heading_rad)
        vy = speed_mps * math.sin(heading_rad)

        t_start = time.time()
        self.drive(vx, vy, 0.0)

        while time.time() - t_start < timeout:
            curr = self.get_odometry()
            dist = math.hypot(curr.x - start_x, curr.y - start_y)
            if dist >= abs(distance_m):
                self.stop()
                return True
            time.sleep(0.02)

        self.stop()
        return False

    def rotate_angle(self, angle_rad: float, speed_rad_s: float = 1.0, timeout: float = 15.0) -> bool:
        """
        Автономный маневр: повернуться на месте на заданный угол (рад).
        Положительный угол — против часовой стрелки.
        """
        start_odom = self.get_odometry()
        start_th = start_odom.theta
        
        target_diff = abs(angle_rad)
        direction = 1.0 if angle_rad > 0 else -1.0
        w = abs(speed_rad_s) * direction

        t_start = time.time()
        self.drive(0.0, 0.0, w)

        while time.time() - t_start < timeout:
            curr = self.get_odometry()
            d_th = abs(curr.theta - start_th)
            if d_th >= target_diff:
                self.stop()
                return True
            time.sleep(0.02)

        self.stop()
        return False

    # =========================================================================
    # Внутренняя реализация потоков и протокола
    # =========================================================================

    def _send_raw(self, cmd_str: str):
        """Потокобезопасная отправка строки команды в Serial."""
        if not self._is_connected or not self._ser:
            return
        if not cmd_str.endswith('\n'):
            cmd_str += '\n'
        with self._ser_lock:
            try:
                self._ser.write(cmd_str.encode('utf-8'))
            except Exception:
                pass

    def _heartbeat_loop(self):
        """Фоновый поток поддержания соединения для аппаратного Watchdog (10 Гц)."""
        while not self._stop_event.is_set():
            if self._is_connected and self._is_moving:
                self._send_raw(self._target_cmd)
            time.sleep(0.1)

    def _telemetry_reader_loop(self):
        """Фоновый поток чтения телеметрии от ESP32 (20 Гц) и расчета одометрии."""
        while not self._stop_event.is_set():
            line = ""
            try:
                with self._ser_lock:
                    if self._ser and self._ser.in_waiting > 0:
                        line = self._ser.readline().decode('utf-8', errors='ignore').strip()
            except Exception:
                pass

            if not line:
                time.sleep(0.005)
                continue

            # Разбор пакета одометрии: "o <p1> <p2> <p3> <s1> <s2> <s3>"
            if line.startswith('o '):
                self._last_telemetry_time = time.time()
                try:
                    parts = line[2:].split()
                    if len(parts) >= 6:
                        p1, p2, p3 = int(parts[0]), int(parts[1]), int(parts[2])
                        s1, s2, s3 = int(parts[3]), int(parts[4]), int(parts[5])
                        self._process_odometry_update(p1, p2, p3, s1, s2, s3)
                except (ValueError, IndexError):
                    pass

    def _process_odometry_update(self, p1: int, p2: int, p3: int, s1: int, s2: int, s3: int):
        """Прямая кинематика 3-колесной платформы и расчет мировой одометрии."""
        now = time.time()

        with self._odom_lock:
            self._odom.wheel_steps = (p1, p2, p3)
            self._odom.wheel_speeds = (s1, s2, s3)
            self._odom.timestamp = now

            if self._last_wheel_steps is None:
                self._last_wheel_steps = (p1, p2, p3)
                self._last_odom_time = now
                return

            dt = now - self._last_odom_time
            if dt <= 0:
                dt = 0.05

            # Дельты шагов за такт
            dp1 = (p1 - self._last_wheel_steps[0]) * self.config.inv_m1
            dp2 = (p2 - self._last_wheel_steps[1]) * self.config.inv_m2
            dp3 = (p3 - self._last_wheel_steps[2]) * self.config.inv_m3

            self._last_wheel_steps = (p1, p2, p3)
            self._last_odom_time = now

            # Перевод шагов в линейное смещение колес (метры)
            ds1 = dp1 * self._meters_per_step
            ds2 = dp2 * self._meters_per_step
            ds3 = dp3 * self._meters_per_step

            # Прямая кинематика для 3-колесной Omni базы:
            # Матрица перехода от смещения колес (ds1, ds2, ds3) к смещению робота (dx, dy, dtheta)
            L = self.config.base_radius
            
            # Локальные смещения в СК робота:
            dx = (2.0 / 3.0) * ds1 - (1.0 / 3.0) * ds2 - (1.0 / 3.0) * ds3
            dy = (1.0 / math.sqrt(3.0)) * (ds3 - ds2)
            dtheta = (1.0 / (3.0 * L)) * (ds1 + ds2 + ds3)

            # Мгновенные скорости тела робота
            self._odom.vx = dx / dt
            self._odom.vy = dy / dt
            self._odom.omega = dtheta / dt

            # Интегрирование позы в мировой системе координат:
            th_mid = self._odom.theta + (dtheta / 2.0)
            self._odom.x += dx * math.cos(th_mid) - dy * math.sin(th_mid)
            self._odom.y += dx * math.sin(th_mid) + dy * math.cos(th_mid)
            self._odom.theta = (self._odom.theta + dtheta + math.pi) % (2.0 * math.pi) - math.pi

            odom_snapshot = OdometryData(
                x=self._odom.x,
                y=self._odom.y,
                theta=self._odom.theta,
                vx=self._odom.vx,
                vy=self._odom.vy,
                omega=self._odom.omega,
                wheel_steps=self._odom.wheel_steps,
                wheel_speeds=self._odom.wheel_speeds,
                timestamp=now
            )

        # Вызов внешних коллбэков (например, для публикации в ROS 2)
        for cb in self._odom_callbacks:
            try:
                cb(odom_snapshot)
            except Exception:
                pass

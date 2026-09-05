"""
Пример использования TermitRobotAPI
===================================
Демонстрация автономного управления платформой Termit Omni:
1. Подключение и получение портов
2. Подписка на поток одометрии в реальном времени (20 Гц)
3. Управление движением тела робота (Twist: вперед/назад/стрейф/разворот)
4. Раздельное управление скоростями отдельных моторов
5. Проверка работы аппаратного Watchdog
6. Управление удержанием (тишина в покое)
"""

import time
from termit_api import TermitRobotAPI, RobotConfig, HoldMode, OdometryData

def on_odometry_received(odom: OdometryData):
    """Коллбэк, вызываемый при получении каждого пакета одометрии (20 Гц)."""
    # Печатаем позу робота (координаты X, Y в метрах и угол в градусах)
    deg = odom.theta * 180.0 / 3.14159265
    print(f"\r📍 Поза: X={odom.x:+.3f}м, Y={odom.y:+.3f}м, Th={deg:+.1f}° | Скорости: Vx={odom.vx:+.2f} Vy={odom.vy:+.2f} W={odom.omega:+.2f} | Шаги: {odom.wheel_steps}", end="", flush=True)

def main():
    print("=" * 70)
    print("🤖 ДЕМОНСТРАЦИЯ TERMIT ROBOT API (Raspberry Pi / Onboard Computer)")
    print("=" * 70)

    # 1. Создаем экземпляр API с конфигурацией робота
    config = RobotConfig(
        wheel_radius=0.030,      # Радиус колеса 30 мм
        base_radius=0.122,       # База 122 мм
        steps_per_rev=1600,      # 1600 микрошагов на оборот
        watchdog_timeout_ms=500  # Стоп моторов если нет связи 500 мс
    )
    robot = TermitRobotAPI(config)

    # Поиск доступных портов
    ports = robot.list_available_ports()
    print(f"Доступные порты: {ports}")
    if not ports:
        print("❌ Нет доступных COM/Serial портов!")
        return

    # 2. Подключение
    target_port = ports[0]
    print(f"Подключение к {target_port} на скорости 115200...")
    robot.connect(port=target_port)
    print("✅ Успешно подключено к контроллеру!")

    # 3. Подписка на поток одометрии
    robot.add_odometry_callback(on_odometry_received)
    robot.reset_odometry()
    time.sleep(0.5)

    try:
        # 4. ТЕСТ 1: Высокоуровневое движение (drive)
        print("\n\n--- [1] ТЕСТ: Движение тела робота вперед (0.15 м/с на 1.5 сек) ---")
        robot.drive(vx=0.0, vy=0.15, omega=0.0)
        time.sleep(1.5)
        robot.stop()
        time.sleep(1.0)

        print("\n\n--- [2] ТЕСТ: Боковой стрейф вправо (0.15 м/с на 1.5 сек) ---")
        robot.drive(vx=0.15, vy=0.0, omega=0.0)
        time.sleep(1.5)
        robot.stop()
        time.sleep(1.0)

        print("\n\n--- [3] ТЕСТ: Разворот на месте (0.8 рад/с на 1.5 сек) ---")
        robot.drive(vx=0.0, vy=0.0, omega=0.8)
        time.sleep(1.5)
        robot.stop()
        time.sleep(1.0)

        # 5. ТЕСТ 2: Раздельное управление моторами (Low-level)
        print("\n\n--- [4] ТЕСТ: Низкоуровневое раздельное управление моторами ---")
        print("Вращаем только Мотор 1 (Передний) со скоростью 800 шагов/сек...")
        robot.set_motor_speeds(m1_steps_s=800, m2_steps_s=0, m3_steps_s=0)
        time.sleep(1.2)
        robot.stop()
        time.sleep(1.0)

        # 6. ТЕСТ 3: Проверка режима тишины (Auto-Sleep)
        print("\n\n--- [5] ТЕСТ: Проверка режима авто-сна (тишина) ---")
        print("Робот стоит. Через 1.5 секунды ток снимется и наступит полная тишина...")
        robot.set_holding_mode(HoldMode.AUTO_SLEEP)
        time.sleep(2.0)
        print("\nПроверяем моментальное пробуждение...")
        robot.drive(vx=0.0, vy=-0.1, omega=0.0)
        time.sleep(0.8)
        robot.stop()
        print("\nОстановлен.")

        # 7. Финальный снимок одометрии
        final_odom = robot.get_odometry()
        print("\n\n" + "=" * 70)
        print("📊 ИТОГОВАЯ ОДОМЕТРИЯ С МОМЕНТА СТАРТА:")
        print(f"X:      {final_odom.x:+.4f} м")
        print(f"Y:      {final_odom.y:+.4f} м")
        print(f"Theta:  {final_odom.theta * 180.0 / 3.14159265:+.2f} градусов")
        print(f"Шаги M1:{final_odom.wheel_steps[0]}")
        print(f"Шаги M2:{final_odom.wheel_steps[1]}")
        print(f"Шаги M3:{final_odom.wheel_steps[2]}")
        print("=" * 70)

    finally:
        print("\nОтключение от робота...")
        robot.disconnect()
        print("Завершено.")

if __name__ == '__main__':
    main()

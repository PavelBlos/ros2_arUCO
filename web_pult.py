"""
Termit Omni - Web Pult powered by TermitRobotAPI
================================================
Веб-интерфейс управления платформой Termit Omni на базе единого termit_api.py.
Поддерживает управление с ПК/планшета/смартфона:
- Живая одометрия (X, Y в метрах, угол в градусах, шаги моторов)
- Сенсорный джойстик 360° и клавиатура WASD/QE/Space
- Переключение режимов удержания (Авто-сон 0 дБ / Блокировка вала)
- Микротесты моторов
- Калибровка кинематики и инверсий на лету
"""

import os
import sys
import time
import json
import math
import threading
from urllib.parse import urlparse, parse_qs
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler

from termit_api import TermitRobotAPI, RobotConfig, HoldMode, OdometryData

PORT = 5000

# Создаем глобальный экземпляр API
config = RobotConfig(
    wheel_radius=0.030,
    base_radius=0.122,
    steps_per_rev=1600,
    max_linear_speed=0.6,
    max_angular_speed=3.5,
    watchdog_timeout_ms=500
)
robot = TermitRobotAPI(config)
current_speed_pct = 50


HTML_PAGE = """<!DOCTYPE html>
<html lang="ru">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Termit Omni - Пульт управления (API)</title>
    <style>
        :root {
            --bg: #0d1117;
            --card-bg: rgba(22, 27, 34, 0.9);
            --accent: #58a6ff;
            --accent-green: #2ea043;
            --accent-red: #f85149;
            --accent-orange: #d29922;
            --border: rgba(240, 246, 252, 0.1);
            --text: #c9d1d9;
            --text-title: #f0f6fc;
        }

        * {
            box-sizing: border-box;
            user-select: none;
            -webkit-user-select: none;
            margin: 0;
            padding: 0;
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
        }

        body {
            background-color: var(--bg);
            color: var(--text);
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 12px;
            touch-action: manipulation;
        }

        .container {
            width: 100%;
            max-width: 480px;
            display: flex;
            flex-direction: column;
            gap: 12px;
        }

        .header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            background: var(--card-bg);
            padding: 10px 16px;
            border-radius: 12px;
            border: 1px solid var(--border);
        }

        .header h1 {
            font-size: 16px;
            color: var(--text-title);
            display: flex;
            align-items: center;
            gap: 8px;
        }

        .card {
            background: var(--card-bg);
            border-radius: 12px;
            border: 1px solid var(--border);
            padding: 12px 16px;
            display: flex;
            flex-direction: column;
            gap: 10px;
        }

        .card-title {
            font-size: 13px;
            font-weight: 600;
            color: var(--text-title);
            display: flex;
            justify-content: space-between;
            align-items: center;
        }

        .status-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            display: inline-block;
            margin-right: 6px;
        }
        .status-dot.online { background-color: var(--accent-green); box-shadow: 0 0 6px var(--accent-green); }
        .status-dot.offline { background-color: var(--accent-red); }

        .connection-row {
            display: flex;
            gap: 8px;
        }

        select {
            flex: 1;
            background: #21262d;
            border: 1px solid var(--border);
            color: var(--text-title);
            padding: 8px 12px;
            border-radius: 8px;
            font-size: 13px;
            outline: none;
        }

        button {
            border: none;
            border-radius: 8px;
            padding: 8px 14px;
            font-size: 13px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.1s ease;
            display: inline-flex;
            align-items: center;
            justify-content: center;
            gap: 6px;
        }
        button:active { transform: scale(0.96); }

        .btn-primary { background: var(--accent); color: #0d1117; }
        .btn-danger { background: var(--accent-red); color: #ffffff; }
        .btn-secondary { background: #21262d; color: var(--text-title); border: 1px solid var(--border); }
        .btn-micro { background: #30363d; color: #58a6ff; font-size: 11px; padding: 5px 8px; border-radius: 6px; }

        /* Одометрия */
        .odom-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 8px;
            background: #161b22;
            padding: 10px;
            border-radius: 8px;
            border: 1px solid var(--border);
            text-align: center;
        }
        .odom-box-title { font-size: 10px; color: #8b949e; text-transform: uppercase; }
        .odom-box-val { font-size: 15px; font-weight: bold; color: var(--accent); margin-top: 2px; }

        .slider-row {
            display: flex;
            align-items: center;
            gap: 10px;
            font-size: 11px;
        }
        .slider-row input[type="range"] {
            flex: 1;
            accent-color: var(--accent);
        }

        .inversion-row {
            display: flex;
            justify-content: space-around;
            background: #161b22;
            padding: 6px 8px;
            border-radius: 6px;
            font-size: 11px;
        }
        .inversion-row label {
            display: flex;
            align-items: center;
            gap: 4px;
            cursor: pointer;
        }

        /* Microtest rows */
        .motor-row {
            display: flex;
            align-items: center;
            justify-content: space-between;
            background: #161b22;
            padding: 6px 10px;
            border-radius: 8px;
            border: 1px solid var(--border);
        }
        .motor-info {
            font-size: 11px;
            font-weight: 600;
        }
        .motor-info small {
            display: block;
            color: #8b949e;
            font-weight: normal;
            font-size: 9px;
        }
        .motor-btns {
            display: flex;
            gap: 6px;
        }

        /* D-Pad */
        .dpad-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 8px;
            max-width: 260px;
            margin: 0 auto;
        }
        .dpad-btn {
            background: #21262d;
            border: 1px solid var(--border);
            color: var(--text-title);
            height: 50px;
            font-size: 18px;
            border-radius: 10px;
        }
        .dpad-btn:active { background: var(--accent); color: #0d1117; }

        /* Touch Joystick */
        .joystick-container {
            width: 170px;
            height: 170px;
            background: radial-gradient(circle, #21262d 0%, #161b22 100%);
            border: 2px solid var(--border);
            border-radius: 50%;
            margin: 4px auto;
            position: relative;
            touch-action: none;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .joystick-knob {
            width: 54px;
            height: 54px;
            background: var(--accent);
            border-radius: 50%;
            box-shadow: 0 0 12px rgba(88, 166, 255, 0.4);
            position: absolute;
            pointer-events: none;
            transition: transform 0.02s linear;
        }

        .keyboard-hint {
            font-size: 11px;
            color: #8b949e;
            text-align: center;
        }
        .keyboard-hint kbd {
            background: #21262d;
            border: 1px solid var(--border);
            padding: 1px 4px;
            border-radius: 4px;
            color: #f0f6fc;
        }
    </style>
</head>
<body>

<div class="container">
    <div class="header">
        <h1>🤖 Termit Omni Pilot <span style="font-size: 10px; background: #238636; padding: 2px 6px; border-radius: 10px;">API v1</span></h1>
        <button class="btn-secondary" style="padding: 5px 10px; font-size: 11px;" onclick="toggleHoldMode()" id="btn-hold-mode">🔇 Авто-сон (Тишина)</button>
    </div>

    <!-- 1. ПОДКЛЮЧЕНИЕ -->
    <div class="card">
        <div class="card-title">
            <span><span class="status-dot" id="status-dot"></span>ESP32 Serial (UART)</span>
            <span id="conn-text" style="font-size: 11px; font-weight: normal;">Отключено</span>
        </div>
        <div class="connection-row">
            <select id="port-select">
                <option value="">Поиск портов...</option>
            </select>
            <button class="btn-secondary" id="btn-refresh">🔄</button>
            <button class="btn-primary" id="btn-connect">Подключить</button>
        </div>
    </div>

    <!-- 2. ЖИВАЯ ОДОМЕТРИЯ В РЕАЛЬНОМ ВРЕМЕНИ -->
    <div class="card">
        <div class="card-title">
            <span>Живая одометрия (20 Гц)</span>
            <button class="btn-secondary" style="padding: 2px 8px; font-size: 11px;" onclick="resetOdom()">🔄 В ноль</button>
        </div>
        <div class="odom-grid">
            <div>
                <div class="odom-box-title">Координата X</div>
                <div class="odom-box-val" id="odom-x">0.000 м</div>
            </div>
            <div>
                <div class="odom-box-title">Координата Y</div>
                <div class="odom-box-val" id="odom-y">0.000 м</div>
            </div>
            <div>
                <div class="odom-box-title">Угол (Yaw)</div>
                <div class="odom-box-val" id="odom-th">0.0°</div>
            </div>
        </div>
        <div style="font-size: 10px; color: #8b949e; text-align: center;" id="odom-steps">
            Шаги колес: M1=0 | M2=0 | M3=0
        </div>
    </div>

    <!-- 3. СКОРОСТЬ И КАЛИБРОВКА -->
    <div class="card">
        <div class="card-title">
            <span>Скорость и кинематика</span>
            <span id="speed-val" style="color: var(--accent);">50%</span>
        </div>
        <div class="slider-row">
            <span>Скорость:</span>
            <input type="range" id="speed-slider" min="10" max="100" value="50" oninput="updateSpeed(this.value)">
        </div>
        <div class="slider-row">
            <span>Боковой баланс:</span>
            <input type="range" id="side-slider" min="10" max="100" value="50" oninput="updateSideRatio(this.value)">
            <span id="side-val">0.50</span>
        </div>
        <div class="inversion-row">
            <label><input type="checkbox" id="inv1" onchange="updateInversions()"> Реверс M1</label>
            <label><input type="checkbox" id="inv2" onchange="updateInversions()"> Реверс M2</label>
            <label><input type="checkbox" id="inv3" onchange="updateInversions()"> Реверс M3</label>
        </div>
    </div>

    <!-- 4. ДВИЖЕНИЕ -->
    <div class="card">
        <div class="card-title">
            <span>Управление платформой</span>
            <button class="btn-danger" style="padding: 3px 8px; font-size: 11px;" onclick="emergencyStop()">СТОП (Space)</button>
        </div>

        <div class="dpad-grid">
            <button class="dpad-btn" onpointerdown="setTarget(0,0,1)" onpointerup="setTarget(0,0,0)" onpointerleave="setTarget(0,0,0)">↺ Q</button>
            <button class="dpad-btn" onpointerdown="setTarget(0,1,0)" onpointerup="setTarget(0,0,0)" onpointerleave="setTarget(0,0,0)">⬆️ W</button>
            <button class="dpad-btn" onpointerdown="setTarget(0,0,-1)" onpointerup="setTarget(0,0,0)" onpointerleave="setTarget(0,0,0)">↻ E</button>

            <button class="dpad-btn" onpointerdown="setTarget(-1,0,0)" onpointerup="setTarget(0,0,0)" onpointerleave="setTarget(0,0,0)">⬅️ A</button>
            <button class="dpad-btn" onpointerdown="setTarget(0,0,0)" style="background: var(--accent-red); color: white;">⏹</button>
            <button class="dpad-btn" onpointerdown="setTarget(1,0,0)" onpointerup="setTarget(0,0,0)" onpointerleave="setTarget(0,0,0)">➡️ D</button>

            <button class="dpad-btn" onpointerdown="setTarget(-0.7,-0.7,0)" onpointerup="setTarget(0,0,0)" onpointerleave="setTarget(0,0,0)">↙️</button>
            <button class="dpad-btn" onpointerdown="setTarget(0,-1,0)" onpointerup="setTarget(0,0,0)" onpointerleave="setTarget(0,0,0)">⬇️ S</button>
            <button class="dpad-btn" onpointerdown="setTarget(0.7,-0.7,0)" onpointerup="setTarget(0,0,0)" onpointerleave="setTarget(0,0,0)">↘️</button>
        </div>

        <div class="keyboard-hint">
            Клавиатура: <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> — ход, <kbd>Q</kbd><kbd>E</kbd> — разворот, <kbd>Space</kbd> — экстренный стоп
        </div>
    </div>

    <!-- 5. СЕНСОРНЫЙ ДЖОЙСТИК -->
    <div class="card">
        <div class="card-title">Сенсорный 360° джойстик</div>
        <div class="joystick-container" id="joy-zone">
            <div class="joystick-knob" id="joy-knob"></div>
        </div>
    </div>

    <!-- 6. МИКРОТЕСТЫ МОТОРОВ -->
    <div class="card">
        <div class="card-title">
            <span>Микротест моторов</span>
        </div>

        <div class="motor-row">
            <div class="motor-info">Мотор 1 (F)<small>CLK 33, CW 32</small></div>
            <div class="motor-btns">
                <button class="btn-micro" onclick="microStep(0, -200)">◀ 200</button>
                <button class="btn-micro" onclick="microStep(0, 200)">200 ▶</button>
                <button class="btn-micro" onclick="microStep(0, 1600)" title="1 оборот">🔄 1 об</button>
            </div>
        </div>

        <div class="motor-row">
            <div class="motor-info">Мотор 2 (R)<small>CLK 23, CW 22</small></div>
            <div class="motor-btns">
                <button class="btn-micro" onclick="microStep(1, -200)">◀ 200</button>
                <button class="btn-micro" onclick="microStep(1, 200)">200 ▶</button>
                <button class="btn-micro" onclick="microStep(1, 1600)" title="1 оборот">🔄 1 об</button>
            </div>
        </div>

        <div class="motor-row">
            <div class="motor-info">Мотор 3 (L)<small>CLK 19, CW 18</small></div>
            <div class="motor-btns">
                <button class="btn-micro" onclick="microStep(2, -200)">◀ 200</button>
                <button class="btn-micro" onclick="microStep(2, 200)">200 ▶</button>
                <button class="btn-micro" onclick="microStep(2, 1600)" title="1 оборот">🔄 1 об</button>
            </div>
        </div>
    </div>
</div>

<script>
    let speed = 50;
    let isConnected = false;

    // Желаемые скорости (Target)
    let targetVx = 0, targetVy = 0, targetW = 0;
    let lastSentVx = 0, lastSentVy = 0, lastSentW = 0;
    let isFetchInFlight = false;

    // Высокоскоростной неблокирующий цикл отправки (25 раз в сек = каждые 40 мс)
    setInterval(() => {
        if (!isConnected && targetVx === 0 && targetVy === 0 && targetW === 0) return;
        if (isFetchInFlight) return;

        const isMoving = (targetVx !== 0 || targetVy !== 0 || targetW !== 0);
        const stateChanged = (targetVx !== lastSentVx || targetVy !== lastSentVy || targetW !== lastSentW);

        if (stateChanged || isMoving) {
            isFetchInFlight = true;
            lastSentVx = targetVx;
            lastSentVy = targetVy;
            lastSentW = targetW;

            fetch(`/drive?vx=${targetVx}&vy=${targetVy}&w=${targetW}`)
                .finally(() => {
                    isFetchInFlight = false;
                });
        }
    }, 40);

    // Фоновое обновление одометрии (5 раз в сек = каждые 200 мс)
    setInterval(() => {
        if (!isConnected) return;
        fetch('/odom')
            .then(res => res.json())
            .then(data => {
                document.getElementById('odom-x').innerText = (data.x >= 0 ? '+' : '') + data.x.toFixed(3) + ' м';
                document.getElementById('odom-y').innerText = (data.y >= 0 ? '+' : '') + data.y.toFixed(3) + ' м';
                document.getElementById('odom-th').innerText = (data.theta_deg >= 0 ? '+' : '') + data.theta_deg.toFixed(1) + '°';
                document.getElementById('odom-steps').innerText = `Шаги: M1=${data.steps[0]} | M2=${data.steps[1]} | M3=${data.steps[2]}`;
            })
            .catch(() => {});
    }, 200);

    function setTarget(vx, vy, w) {
        targetVx = vx;
        targetVy = vy;
        targetW = w;
    }

    function emergencyStop() {
        targetVx = 0; targetVy = 0; targetW = 0;
        lastSentVx = 0; lastSentVy = 0; lastSentW = 0;
        isFetchInFlight = false;
        fetch('/drive?vx=0&vy=0&w=0');
    }

    function resetOdom() {
        fetch('/reset_odom');
    }

    function updateSpeed(val) {
        speed = parseInt(val);
        document.getElementById('speed-val').innerText = speed + '%';
        fetch(`/set_speed?speed=${speed}`);
    }

    function updateSideRatio(val) {
        const ratio = (parseInt(val) / 100).toFixed(2);
        document.getElementById('side-val').innerText = ratio;
        fetch(`/set_calibration?side_ratio=${ratio}`);
    }

    function updateInversions() {
        const i1 = document.getElementById('inv1').checked ? -1 : 1;
        const i2 = document.getElementById('inv2').checked ? -1 : 1;
        const i3 = document.getElementById('inv3').checked ? -1 : 1;
        fetch(`/set_inversion?m1=${i1}&m2=${i2}&m3=${i3}`);
    }

    let isAutoSleep = true;
    function toggleHoldMode() {
        isAutoSleep = !isAutoSleep;
        const btn = document.getElementById('btn-hold-mode');
        if (isAutoSleep) {
            btn.innerText = '🔇 Авто-сон (Тишина)';
            btn.className = 'btn-secondary';
            fetch('/set_hold_mode?mode=1');
        } else {
            btn.innerText = '🔒 Жесткое удержание';
            btn.className = 'btn-primary';
            fetch('/set_hold_mode?mode=0');
        }
    }

    function microStep(motorIdx, steps) {
        fetch(`/microstep?motor=${motorIdx}&steps=${steps}`);
    }

    function fetchPorts() {
        fetch('/ports')
            .then(res => res.json())
            .then(data => {
                const sel = document.getElementById('port-select');
                sel.innerHTML = '';
                if (data.ports.length === 0) {
                    sel.innerHTML = '<option value="">Нет портов</option>';
                } else {
                    data.ports.forEach(p => {
                        const opt = document.createElement('option');
                        opt.value = p;
                        opt.innerText = p;
                        sel.appendChild(opt);
                    });
                }
                if (data.connected_port) {
                    sel.value = data.connected_port;
                    setConnectedState(true, data.connected_port);
                }
            });
    }

    function setConnectedState(connected, port) {
        isConnected = connected;
        const dot = document.getElementById('status-dot');
        const txt = document.getElementById('conn-text');
        const btn = document.getElementById('btn-connect');

        if (connected) {
            dot.className = 'status-dot online';
            txt.innerText = 'Подключен: ' + port;
            txt.style.color = '#2ea043';
            btn.innerText = 'Отключить';
            btn.className = 'btn-danger';
        } else {
            dot.className = 'status-dot offline';
            txt.innerText = 'Отключено';
            txt.style.color = '#da3633';
            btn.innerText = 'Подключить';
            btn.className = 'btn-primary';
        }
    }

    document.getElementById('btn-refresh').addEventListener('click', fetchPorts);

    document.getElementById('btn-connect').addEventListener('click', () => {
        if (isConnected) {
            fetch('/disconnect').then(res => res.json()).then(() => setConnectedState(false, ''));
        } else {
            const port = document.getElementById('port-select').value;
            if (!port) return;
            fetch(`/connect?port=${encodeURIComponent(port)}`)
                .then(res => res.json())
                .then(data => {
                    if (data.success) setConnectedState(true, port);
                    else alert('Ошибка подключения: ' + data.error);
                });
        }
    });

    // Клавиатура (WASD / QE / Space)
    const keys = {};
    window.addEventListener('keydown', (e) => {
        if (e.code === 'Space') {
            e.preventDefault();
            for (let k in keys) delete keys[k];
            resetJoy();
            emergencyStop();
            return;
        }
        if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.code)) {
            e.preventDefault();
        }
        if (e.repeat) return;
        keys[e.code] = true;
        calcKeyboardState();
    });
    window.addEventListener('keyup', (e) => {
        if (e.code === 'Space') return;
        delete keys[e.code];
        calcKeyboardState();
    });

    function calcKeyboardState() {
        let vx = 0, vy = 0, w = 0;
        if (keys['KeyW'] || keys['ArrowUp']) vy += 1;
        if (keys['KeyS'] || keys['ArrowDown']) vy -= 1;
        if (keys['KeyA'] || keys['ArrowLeft']) vx -= 1;
        if (keys['KeyD'] || keys['ArrowRight']) vx += 1;
        if (keys['KeyQ']) w += 1;
        if (keys['KeyE']) w -= 1;
        setTarget(vx, vy, w);
    }

    // Джойстик 360°
    const zone = document.getElementById('joy-zone');
    const knob = document.getElementById('joy-knob');
    const maxR = 60;
    let joyActive = false, joyCX = 0, joyCY = 0;

    function resetJoy() {
        joyActive = false;
        knob.style.transform = `translate(0px, 0px)`;
        targetVx = 0;
        targetVy = 0;
        targetW = 0;
        lastSentVx = 0;
        lastSentVy = 0;
        lastSentW = 0;
        isFetchInFlight = false;
        fetch('/drive?vx=0&vy=0&w=0');
    }

    function handleJoy(clientX, clientY) {
        const dx = clientX - joyCX;
        const dy = clientY - joyCY;
        const dist = Math.sqrt(dx * dx + dy * dy);

        if (dist < 8) {
            knob.style.transform = `translate(0px, 0px)`;
            setTarget(0, 0, 0);
            return;
        }

        const angle = Math.atan2(dy, dx);
        const r = Math.min(dist, maxR);
        const kx = r * Math.cos(angle);
        const ky = r * Math.sin(angle);
        knob.style.transform = `translate(${kx}px, ${ky}px)`;

        const vx = parseFloat((kx / maxR).toFixed(2));
        const vy = parseFloat((-ky / maxR).toFixed(2));
        setTarget(vx, vy, 0);
    }

    zone.addEventListener('pointerdown', (e) => {
        joyActive = true;
        const rect = zone.getBoundingClientRect();
        joyCX = rect.left + rect.width / 2;
        joyCY = rect.top + rect.height / 2;
        handleJoy(e.clientX, e.clientY);
    });

    window.addEventListener('pointermove', (e) => {
        if (joyActive) handleJoy(e.clientX, e.clientY);
    });

    window.addEventListener('pointerup', () => {
        if (joyActive) resetJoy();
    });

    fetchPorts();
</script>

</body>
</html>
"""

class FastWebPultHandler(SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)

        if path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))

        elif path == '/ports':
            ports = TermitRobotAPI.list_available_ports()
            self.send_json({"ports": ports, "connected_port": robot._port_name if robot.is_connected else None})

        elif path == '/connect':
            port = qs.get('port', [''])[0]
            try:
                success = robot.connect(port=port)
                self.send_json({"success": success, "error": None})
            except Exception as e:
                self.send_json({"success": False, "error": str(e)})

        elif path == '/disconnect':
            robot.disconnect()
            self.send_json({"success": True})

        elif path == '/set_speed':
            global current_speed_pct
            current_speed_pct = int(qs.get('speed', [50])[0])
            self.send_json({"status": "ok", "speed": current_speed_pct})

        elif path == '/set_calibration':
            if 'side_ratio' in qs:
                robot.config.side_ratio = float(qs['side_ratio'][0])
            self.send_json({"status": "ok", "side_ratio": robot.config.side_ratio})

        elif path == '/set_inversion':
            robot.config.inv_m1 = int(qs.get('m1', [1])[0])
            robot.config.inv_m2 = int(qs.get('m2', [1])[0])
            robot.config.inv_m3 = int(qs.get('m3', [1])[0])
            self.send_json({"status": "ok"})

        elif path == '/set_hold_mode':
            mode = int(qs.get('mode', [1])[0])
            robot.set_holding_mode(HoldMode.AUTO_SLEEP if mode == 1 else HoldMode.CONTINUOUS_HOLD)
            self.send_json({"status": "ok", "mode": mode})

        elif path == '/microstep':
            motor_idx = int(qs.get('motor', [0])[0])
            steps = int(qs.get('steps', [200])[0])
            robot.microstep(motor_idx, steps)
            self.send_json({"status": "ok", "motor": motor_idx, "steps": steps})

        elif path == '/odom':
            odom = robot.get_odometry()
            self.send_json({
                "x": round(odom.x, 4),
                "y": round(odom.y, 4),
                "theta_deg": round(math.degrees(odom.theta), 1),
                "vx": round(odom.vx, 3),
                "vy": round(odom.vy, 3),
                "w": round(odom.omega, 3),
                "steps": odom.wheel_steps
            })

        elif path == '/reset_odom':
            robot.reset_odometry()
            self.send_json({"status": "ok"})

        elif path == '/drive':
            vx = float(qs.get('vx', [0.0])[0])
            vy = float(qs.get('vy', [0.0])[0])
            w = float(qs.get('w', [0.0])[0])
            
            if vx == 0.0 and vy == 0.0 and w == 0.0:
                robot.stop()
            else:
                # Масштабируем относительные значения (-1..1) в физические м/с по выбранному проценту скорости
                scale = current_speed_pct / 100.0
                vx_mps = vx * scale * robot.config.max_linear_speed
                vy_mps = vy * scale * robot.config.max_linear_speed
                w_rads = w * scale * robot.config.max_angular_speed
                robot.drive(vx_mps, vy_mps, w_rads)
                
            self.send_json({"status": "ok"})

        else:
            self.send_error(404)

    def send_json(self, data):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode('utf-8'))

def main():
    server = ThreadingHTTPServer(('0.0.0.0', PORT), FastWebPultHandler)
    print("=" * 65)
    print(f"Termit Omni Web Pult (Powered by TermitRobotAPI) started!")
    print(f"Open in browser: http://localhost:{PORT}")
    print("=" * 65)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping server...")
        robot.disconnect()

if __name__ == '__main__':
    main()

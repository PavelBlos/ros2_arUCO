import cv2
import time
import sys
import argparse
import os
import threading
from flask import Flask, Response

# Глобальные переменные для веб-трансляции
current_frame = None
is_recording = True

app = Flask(__name__)

def gen_frames():
    global current_frame, is_recording
    while is_recording:
        if current_frame is not None:
            # Сжимаем текущий кадр в JPEG
            ret, buffer = cv2.imencode('.jpg', current_frame)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.033)  # Ограничение частоты трансляции до ~30 FPS

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return '''
    <body style="margin:0; background:#111; display:flex; justify-content:center; align-items:center; height:100vh; font-family:sans-serif;">
        <div style="color:red; position:absolute; top:20px; background:rgba(0,0,0,0.8); padding:10px 20px; border-radius:5px; font-weight:bold; font-size: 1.2em; animation: blinker 1s linear infinite; border: 1px solid red;">
            🔴 ИДЕТ ЗАПИСЬ И ТРАНСЛЯЦИЯ В РЕАЛЬНОМ ВРЕМЕНИ
        </div>
        <img src="/video_feed" style="max-width:100%; max-height:100%; object-fit:contain;">
        <style>
            @keyframes blinker { 50% { opacity: 0.2; } }
        </style>
    </body>
    '''

def run_server():
    # Отключаем логгирование Flask в консоль, чтобы не засорять вывод записи
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    app.run(host='0.0.0.0', port=5000, threaded=True)

def main():
    parser = argparse.ArgumentParser(description="Record video from Raspberry Pi camera with optional web streaming")
    parser.add_argument("duration", type=float, help="Duration in seconds")
    parser.add_argument("resolution", type=str, help="Resolution (e.g. 640x480 or 1280x720)")
    parser.add_argument("fps", type=float, help="FPS (e.g. 30)")
    parser.add_argument("output", type=str, help="Output file path")
    parser.add_argument("--stream", action="store_true", help="Enable live web stream during recording")

    args = parser.parse_args()

    try:
        width, height = map(int, args.resolution.split('x'))
    except ValueError:
        print("Error: Resolution must be in format WxH, e.g. 640x480")
        sys.exit(1)

    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # Убеждаемся, что выходная папка существует
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(args.output, fourcc, args.fps, (width, height))

    global current_frame, is_recording

    # Запуск фонового веб-сервера, если указан флаг --stream
    if args.stream:
        is_recording = True
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        print("\n==================================================================")
        print("🌐 ВЕБ-ТРАНСЛЯЦИЯ ЗАПУЩЕНА!")
        print("Откройте в браузере вашего ПК: http://192.168.31.215:5000")
        print("Вам дается 5 секунд, чтобы открыть страницу и навести камеру...")
        print("==================================================================\n")
        
        # Обратный отсчет перед началом записи, чтобы пользователь успел открыть браузер
        for i in range(5, 0, -1):
            print(f"Запись начнется через {i}...")
            time.sleep(1)

    print(f"\n⏺️ Запись пошла: {args.output} ({width}x{height} @ {args.fps} FPS) на {args.duration} секунд...")
    start_time = time.time()
    frames_recorded = 0
    
    while time.time() - start_time < args.duration:
        ret, frame = camera.read()
        if ret:
            # Преобразуем цвета (RGB -> BGR) для корректного сохранения видео
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Обновляем глобальный кадр для веб-стрима
            current_frame = frame.copy()
            
            out.write(frame)
            frames_recorded += 1
        else:
            print("Error: Failed to grab frame.")
            break

    # Сигнализируем о завершении записи для остановки генератора кадров
    is_recording = False
    camera.release()
    out.release()
    print(f"✔️ Запись успешно завершена! Записано кадров: {frames_recorded}\n")

if __name__ == '__main__':
    main()

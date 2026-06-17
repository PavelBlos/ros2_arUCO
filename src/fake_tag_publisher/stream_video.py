import cv2
import time
import sys
import argparse
from flask import Flask, Response

def main():
    parser = argparse.ArgumentParser(description="Stream a recorded video file to browser")
    parser.add_argument("video_path", type=str, help="Path to the video file")
    args = parser.parse_args()

    app = Flask(__name__)

    def gen_frames():
        while True:
            cap = cv2.VideoCapture(args.video_path)
            if not cap.isOpened():
                print(f"Error: Could not open video file: {args.video_path}")
                break
                
            fps = cap.get(cv2.CAP_PROP_FPS)
            # Fallback to 30 FPS if not found in metadata
            delay = 1.0 / fps if fps > 0 else 0.033
            
            while cap.isOpened():
                start_time = time.time()
                ret, frame = cap.read()
                if not ret:
                    break
                
                # Compress frame to JPG
                ret, buffer = cv2.imencode('.jpg', frame)
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                
                # Control streaming speed to match video FPS
                elapsed = time.time() - start_time
                if elapsed < delay:
                    time.sleep(delay - elapsed)
            cap.release()

    @app.route('/video_feed')
    def video_feed():
        return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/')
    def index():
        return f'<body style="margin:0; background:#111; display:flex; justify-content:center; align-items:center; height:100vh;"><div style="color:white; position:absolute; top:10px; font-family:sans-serif; background:rgba(0,0,0,0.6); padding:5px 10px; border-radius:5px;">Streaming: {args.video_path}</div><img src="/video_feed" style="max-width:100%; max-height:100%; object-fit:contain;"></body>'

    app.run(host='0.0.0.0', port=5000, threaded=True)

if __name__ == '__main__':
    main()

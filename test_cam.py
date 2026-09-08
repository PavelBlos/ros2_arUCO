#!/usr/bin/env python3
import subprocess
import os
import cv2
import numpy as np
import time

print("=== 1. Initializing IMX296 pipeline ===")
subprocess.run('v4l2-ctl -d /dev/v4l-subdev0 --set-subdev-fmt pad=0,width=1456,height=1088,code=0x3007', shell=True)
subprocess.run('v4l2-ctl -d /dev/v4l-subdev0 --set-subdev-fmt pad=4,width=1456,height=1088,code=0x3007', shell=True)
subprocess.run("media-ctl -d /dev/media0 -l \"'csi2':4 -> 'rp1-cfe-csi2_ch0':0 [1]\"", shell=True)
subprocess.run('v4l2-ctl -d /dev/video0 --set-fmt-video=width=1456,height=1088,pixelformat=pBAA', shell=True)

print("\n=== 2. Starting real-time streaming pipe ===")
FRAME_SIZE = 1980160
proc = subprocess.Popen(
    ['v4l2-ctl', '-d', '/dev/video0', '--stream-mmap', '--stream-to=-'],
    stdout=subprocess.PIPE,
    stderr=subprocess.DEVNULL,
    bufsize=FRAME_SIZE * 2
)

dict_obj = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100) if hasattr(cv2.aruco, 'Dictionary_get') else cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
params = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create') else cv2.aruco.DetectorParameters()
params.minMarkerPerimeterRate = 0.08
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX') else 1

t0 = time.time()
for i in range(10):
    t_start = time.time()
    buf = proc.stdout.read(FRAME_SIZE)
    if len(buf) < FRAME_SIZE:
        print(f"Incomplete frame: {len(buf)} bytes")
        break
    
    # Быстрая распаковка 10-бит Bayer pBAA в 8-бит Grayscale
    row_data = np.frombuffer(buf, dtype=np.uint8).reshape((1088, 1820))
    gray = np.empty((1088, 1456), dtype=np.uint8)
    gray[:, 0::4] = row_data[:, 0::5]
    gray[:, 1::4] = row_data[:, 1::5]
    gray[:, 2::4] = row_data[:, 2::5]
    gray[:, 3::4] = row_data[:, 3::5]
    
    # Срезаем каждый второй пиксель для мгновенной обработки 30 FPS (728x544)
    gray_sub = gray[::2, ::2]
    # Ресайз до 640x480 (разрешение калибровки камеры)
    gray_640 = cv2.resize(gray_sub, (640, 480), interpolation=cv2.INTER_LINEAR)
    if i == 0:
        cv2.imwrite('/tmp/test_cam_frame.jpg', gray_640)
    
    # Детекция меток ArUco
    corners, ids, _ = cv2.aruco.detectMarkers(gray_640, dict_obj, parameters=params)
    dt = (time.time() - t_start) * 1000
    if ids is not None:
        tag_details = []
        for tid, c in zip(ids.flatten(), corners):
            pts = c[0]
            perim = cv2.arcLength(pts, True)
            area = cv2.contourArea(pts)
            tag_details.append(f"id={tid}(perim={perim:.1f},area={area:.1f})")
        print(f"Frame {i}: tags: {', '.join(tag_details)}")
    else:
        print(f"Frame {i}: no tags")

proc.terminate()
proc.wait()
total_fps = 10.0 / (time.time() - t0)
print(f"\n🎉 Streaming success! Achieved FPS: {total_fps:.1f}")












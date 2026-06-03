import cv2
import numpy as np
import os
import time

def benchmark():
    video_path = "/home/panik_bel/.gemini/antigravity/scratch/ros2_ws_AGrep/ros2_ws/src/fake_tag_publisher/config/robot_drive.mp4"
    if not os.path.exists(video_path):
        print("Video not found")
        return

    # Load calibration if available, else use dummy values
    camera_matrix = np.array([[524.24, 0, 311.64], [0, 528.10, 266.16], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.array([0.28, -0.89, 0.02, -0.008, 1.16], dtype=np.float32)

    # Initialize detector
    dict_id = cv2.aruco.DICT_4X4_100
    try:
        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        detect_func = lambda img: detector.detectMarkers(img)
    except AttributeError:
        dictionary = cv2.aruco.Dictionary_get(dict_id)
        parameters = cv2.aruco.DetectorParameters_create()
        detect_func = lambda img: cv2.aruco.detectMarkers(img, dictionary, parameters=parameters)

    # Load 100 frames to memory to test speed without disk I/O interference
    cap = cv2.VideoCapture(video_path)
    frames = []
    while len(frames) < 100:
        ret, frame = cap.read()
        if not ret:
            break
        frames.append(frame)
    cap.release()
    print(f"Loaded {len(frames)} frames for benchmarking.")

    # Method 1: Current approach (undistort whole frame, solvePnP with zeros)
    start_time = time.time()
    for frame in frames:
        undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detect_func(gray)
        if ids is not None:
            for i in range(len(ids)):
                obj_pts = np.array([[-0.075, 0.075, 0], [0.075, 0.075, 0], [0.075, -0.075, 0], [-0.075, -0.075, 0]], dtype=np.float32)
                cv2.solvePnP(obj_pts, corners[i][0], camera_matrix, np.zeros(5), flags=cv2.SOLVEPNP_ITERATIVE)
    m1_time = time.time() - start_time
    print(f"Method 1 (cv2.undistort): {m1_time:.4f}s ({len(frames)/m1_time:.1f} FPS)")

    # Method 2: Precomputed maps (cv2.remap, solvePnP with zeros)
    h, w = frames[0].shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, new_camera_matrix, (w, h), cv2.CV_16SC2)
    
    start_time = time.time()
    for frame in frames:
        undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detect_func(gray)
        if ids is not None:
            for i in range(len(ids)):
                obj_pts = np.array([[-0.075, 0.075, 0], [0.075, 0.075, 0], [0.075, -0.075, 0], [-0.075, -0.075, 0]], dtype=np.float32)
                cv2.solvePnP(obj_pts, corners[i][0], new_camera_matrix, np.zeros(5), flags=cv2.SOLVEPNP_ITERATIVE)
    m2_time = time.time() - start_time
    print(f"Method 2 (cv2.remap with optimal matrix): {m2_time:.4f}s ({len(frames)/m2_time:.1f} FPS)")

    # Method 3: No image undistortion (detect raw, solvePnP with dist_coeffs)
    start_time = time.time()
    for frame in frames:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detect_func(gray)
        if ids is not None:
            for i in range(len(ids)):
                obj_pts = np.array([[-0.075, 0.075, 0], [0.075, 0.075, 0], [0.075, -0.075, 0], [-0.075, -0.075, 0]], dtype=np.float32)
                cv2.solvePnP(obj_pts, corners[i][0], camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    m3_time = time.time() - start_time
    print(f"Method 3 (No image undistortion, solvePnP with dist_coeffs): {m3_time:.4f}s ({len(frames)/m3_time:.1f} FPS)")

if __name__ == "__main__":
    benchmark()

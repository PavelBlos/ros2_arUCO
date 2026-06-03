import cv2
import numpy as np
import os

def analyze():
    video_path = "/home/panik_bel/.gemini/antigravity/scratch/ros2_ws_AGrep/ros2_ws/src/fake_tag_publisher/config/robot_drive.mp4"
    if not os.path.exists(video_path):
        print(f"Video file not found at: {video_path}")
        return

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Failed to open video file.")
        return

    print("Video opened successfully. Checking dictionaries on the first 100 frames...")
    
    # List of common dictionaries to check
    dicts_to_check = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X5_50": 4, # cv2.aruco.DICT_5X5_50 (5x5 starts at index 4 usually)
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
    }

    # Initialize detectors
    detectors = {}
    for name, dict_id in dicts_to_check.items():
        try:
            # Try OpenCV 4.7+ API
            dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            detectors[name] = lambda img, d=detector: d.detectMarkers(img)
        except AttributeError:
            # Legacy OpenCV API
            dictionary = cv2.aruco.Dictionary_get(dict_id)
            parameters = cv2.aruco.DetectorParameters_create()
            detectors[name] = lambda img, dic=dictionary, p=parameters: cv2.aruco.detectMarkers(img, dic, parameters=p)

    results = {name: [] for name in detectors.keys()}
    frame_idx = 0
    saved_image = False

    while frame_idx < 150:
        ret, frame = cap.read()
        if not ret:
            break
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        for name, detect_func in detectors.items():
            corners, ids, rejected = detect_func(gray)
            if ids is not None and len(ids) > 0:
                detected_ids = [int(i[0]) for i in ids]
                results[name].append((frame_idx, detected_ids))
                
                # Save the first detected frame for visual inspection
                if not saved_image and name in ["DICT_6X6_250", "DICT_6X6_50", "DICT_4X4_50"]:
                    annotated = frame.copy()
                    cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
                    out_path = "/home/panik_bel/.gemini/antigravity/scratch/ros2_ws_AGrep/ros2_ws/src/fake_tag_publisher/config/detected_tag_debug.png"
                    cv2.imwrite(out_path, annotated)
                    print(f"Saved debug image with detection ({name}) to {out_path}")
                    saved_image = True
        
        frame_idx += 1

    print("\n--- Detection Results per Dictionary ---")
    for name, detections in results.items():
        if len(detections) > 0:
            # Count frequency of each ID
            id_counts = {}
            for f, ids in detections:
                for idx in ids:
                    id_counts[idx] = id_counts.get(idx, 0) + 1
            print(f"{name}: detected in {len(detections)} frames. ID counts: {id_counts}")
        else:
            # print(f"{name}: no detections")
            pass

    cap.release()

if __name__ == "__main__":
    analyze()

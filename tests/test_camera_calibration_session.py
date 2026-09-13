import os
import sys

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', 'src', 'fake_tag_publisher', 'fake_tag_publisher'
)))

from camera_calibration_session import CameraCalibrationSession, make_a4_chessboard_svg


def test_a4_board_matches_default_calibration_pattern():
    svg = make_a4_chessboard_svg(8, 6, 25)
    assert 'width="297mm"' in svg
    assert 'height="210mm"' in svg
    assert '8×6 внутренних углов' in svg
    assert svg.count('<rect x=') == 32  # black cells in a 9x7 board


def test_cancel_is_not_overwritten_by_frame_finishing(monkeypatch):
    session = CameraCalibrationSession()
    session.start()

    def finish_after_cancel(*args, **kwargs):
        session.cancel()
        return False, None

    monkeypatch.setattr(cv2, "findChessboardCorners", finish_after_cancel)
    session.process_frame(np.zeros((480, 640, 3), dtype=np.uint8))
    status = session.status()
    assert status["state"] == "cancelled"
    assert "остановлена" in status["guidance"].lower()


def test_solve_and_atomically_apply_camera_calibration(tmp_path):
    session = CameraCalibrationSession()
    session.start(required_frames=25)
    true_k = np.array([[780.0, 0.0, 320.0], [0.0, 785.0, 240.0], [0.0, 0.0, 1.0]])
    distortion = np.zeros(5)
    obj = session._object_template()

    for index in range(25):
        row, col = divmod(index, 5)
        rvec = np.array([0.08 * (row - 2), 0.07 * (col - 2), 0.025 * (index - 12)])
        tvec = np.array([0.05 * (col - 2), 0.04 * (row - 2), 0.75 + 0.025 * ((row + col) % 3)])
        points, _ = cv2.projectPoints(obj, rvec, tvec, true_k, distortion)
        session.object_points.append(obj.copy())
        session.image_points.append(points.astype(np.float32))
        session.signatures.append(np.zeros(4))

    session.image_size = (640, 480)
    session.state = "ready"
    candidate = session.solve()
    assert candidate["frames_used"] == 25
    assert candidate["rms_error_px"] < 0.01
    assert abs(candidate["camera_matrix"][2] - 320.0) < 1.0
    assert abs(candidate["camera_matrix"][5] - 240.0) < 1.0

    target = tmp_path / "camera_info.yaml"
    target.write_text("old: calibration\n", encoding="utf-8")
    session.apply(str(target))
    saved = yaml.safe_load(target.read_text(encoding="utf-8"))
    assert saved["image_width"] == 640
    assert len(saved["camera_matrix"]) == 9
    assert session.status()["state"] == "applied"
    assert (tmp_path / "camera_info.yaml.previous").read_text(encoding="utf-8") == "old: calibration\n"

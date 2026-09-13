"""Live, guided camera calibration from the web remote."""

import math
import os
import shutil
import threading
import time

import cv2
import numpy as np
import yaml


def make_a4_chessboard_svg(inner_cols=8, inner_rows=6, square_mm=25.0):
    """Return an A4 landscape SVG for an OpenCV inner-corner chessboard."""
    cols = int(inner_cols) + 1
    rows = int(inner_rows) + 1
    square = float(square_mm)
    board_w = cols * square
    board_h = rows * square
    if board_w > 287 or board_h > 200:
        raise ValueError("board does not fit on A4 landscape with safe margins")
    x0 = (297.0 - board_w) / 2.0
    y0 = (210.0 - board_h) / 2.0
    cells = []
    for row in range(rows):
        for col in range(cols):
            if (row + col) % 2 == 0:
                cells.append(
                    f'<rect x="{x0 + col * square:.3f}" y="{y0 + row * square:.3f}" '
                    f'width="{square:.3f}" height="{square:.3f}" fill="#000"/>'
                )
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<svg xmlns="http://www.w3.org/2000/svg" width="297mm" height="210mm" '
        'viewBox="0 0 297 210">\n'
        '<rect width="297" height="210" fill="#fff"/>\n'
        + '\n'.join(cells)
        + f'\n<text x="148.5" y="207" text-anchor="middle" font-family="Arial" font-size="3.2">'
          f'Калибровочная доска: {inner_cols}×{inner_rows} внутренних углов, квадрат {square:g} мм. '
          'Печатать 100%, без масштабирования.</text>\n</svg>\n'
    )


class CameraCalibrationSession:
    """Collect diverse chessboard views, solve intrinsics, and stage the result."""

    def __init__(self):
        self._lock = threading.RLock()
        self.reset()

    def reset(self):
        with getattr(self, "_lock", threading.RLock()):
            self.state = "idle"
            self.inner_cols = 8
            self.inner_rows = 6
            self.square_mm = 25.0
            self.required_frames = 25
            self.object_points = []
            self.image_points = []
            self.signatures = []
            self.image_size = None
            self.board_detected = False
            self.sharpness = 0.0
            self.coverage = 0.0
            self.last_capture_at = 0.0
            self.guidance = "Нажмите «Начать» и покажите камере распечатанную шахматную доску."
            self.error = ""
            self.candidate = None

    def start(self, inner_cols=8, inner_rows=6, square_mm=25.0, required_frames=25):
        inner_cols, inner_rows = int(inner_cols), int(inner_rows)
        square_mm, required_frames = float(square_mm), int(required_frames)
        if not 4 <= inner_cols <= 15 or not 4 <= inner_rows <= 12:
            raise ValueError("число внутренних углов должно быть в диапазоне 4..15 × 4..12")
        if not 5.0 <= square_mm <= 60.0:
            raise ValueError("размер квадрата должен быть 5..60 мм")
        if not 10 <= required_frames <= 50:
            raise ValueError("нужно от 10 до 50 кадров")
        with self._lock:
            self.reset()
            self.inner_cols = inner_cols
            self.inner_rows = inner_rows
            self.square_mm = square_mm
            self.required_frames = required_frames
            self.state = "collecting"
            self.guidance = "Держите доску целиком в кадре. Начните по центру, затем перемещайте её к краям и наклоняйте."
        return self.status()

    def cancel(self):
        with self._lock:
            self.state = "cancelled"
            self.guidance = "Калибровка остановлена. Сохранённая калибровка не изменена."

    def _object_template(self):
        obj = np.zeros((self.inner_cols * self.inner_rows, 3), np.float32)
        obj[:, :2] = np.mgrid[0:self.inner_cols, 0:self.inner_rows].T.reshape(-1, 2)
        obj *= self.square_mm / 1000.0
        return obj

    def process_jpeg(self, jpeg_bytes, now=None):
        frame = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return False
        return self.process_frame(frame, now=now)

    def process_frame(self, frame, now=None):
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            if self.state != "collecting":
                return False
            cols, rows = self.inner_cols, self.inner_rows

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
        sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())

        with self._lock:
            # The user may have cancelled while OpenCV was processing this frame.
            if self.state != "collecting":
                return False
            self.board_detected = bool(found)
            self.sharpness = round(sharpness, 1)
            self.image_size = (int(gray.shape[1]), int(gray.shape[0]))
            if not found:
                self.coverage = 0.0
                self.guidance = f"Доска не найдена. Покажите все {cols + 1}×{rows + 1} клеток, без бликов и перекрытий."
                return False

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
            refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            pts = refined.reshape(-1, 2)
            hull_area = float(cv2.contourArea(cv2.convexHull(pts.astype(np.float32))))
            coverage = hull_area / float(gray.shape[0] * gray.shape[1])
            self.coverage = round(coverage * 100.0, 1)
            if coverage < 0.025:
                self.guidance = "Поднесите доску ближе: сейчас она слишком мала для точной калибровки."
                return False
            if sharpness < 45.0:
                self.guidance = "Кадр размыт. Зафиксируйте доску и камеру на секунду."
                return False

            center = pts.mean(axis=0)
            edge = pts[-1] - pts[0]
            angle = math.atan2(float(edge[1]), float(edge[0])) / math.pi
            signature = np.array([
                center[0] / gray.shape[1], center[1] / gray.shape[0],
                math.sqrt(max(coverage, 0.0)), angle,
            ], dtype=np.float64)
            if now - self.last_capture_at < 0.65:
                self.guidance = "Отлично, доска найдена. Медленно смените положение или наклон."
                return False
            if self.signatures and min(float(np.linalg.norm(signature - s)) for s in self.signatures) < 0.055:
                self.guidance = self._next_guidance()
                return False

            self.object_points.append(self._object_template())
            self.image_points.append(refined.copy())
            self.signatures.append(signature)
            self.last_capture_at = now
            if len(self.image_points) >= self.required_frames:
                self.state = "ready"
                self.guidance = f"{self.required_frames} разных кадров собрано. Нажмите «Рассчитать калибровку»."
            else:
                self.guidance = self._next_guidance()
            return True

    def _next_guidance(self):
        n = len(self.signatures)
        if n < 5:
            return "Кадр принят. Держите доску ровно и переместите её в другую часть кадра."
        centers = np.array([s[:2] for s in self.signatures])
        targets = [((0.2, 0.2), "верхний левый угол"), ((0.8, 0.2), "верхний правый угол"),
                   ((0.2, 0.8), "нижний левый угол"), ((0.8, 0.8), "нижний правый угол")]
        target, label = max(targets, key=lambda item: np.min(np.linalg.norm(centers - item[0], axis=1)))
        del target
        if n < 17:
            return f"Кадр принят. Переместите доску ближе к области: {label}."
        return "Кадр принят. Наклоните доску по диагонали и измените расстояние до камеры."

    def solve(self):
        with self._lock:
            if self.state != "ready" or len(self.image_points) < self.required_frames:
                raise ValueError("сначала соберите все необходимые кадры")
            object_points = [p.copy() for p in self.object_points]
            image_points = [p.copy() for p in self.image_points]
            image_size = tuple(self.image_size)

        rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
            object_points, image_points, image_size, None, None
        )
        del rvecs, tvecs
        if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(distortion)):
            raise ValueError("OpenCV вернул некорректные параметры")
        if rms > 1.5:
            raise ValueError(f"слишком большая ошибка калибровки: {rms:.3f} px")
        width, height = image_size
        warnings = []
        if abs(matrix[0, 2] - width / 2) > width * 0.08 or abs(matrix[1, 2] - height / 2) > height * 0.08:
            warnings.append("оптический центр заметно смещён; проверьте разнообразие кадров")
        candidate = {
            "image_width": width,
            "image_height": height,
            "camera_matrix": matrix.reshape(-1).tolist(),
            "distortion_coefficients": distortion.reshape(-1).tolist(),
            "rms_error_px": round(float(rms), 4),
            "frames_used": len(image_points),
            "warnings": warnings,
        }
        with self._lock:
            self.candidate = candidate
            self.state = "review"
            self.guidance = "Расчёт готов. Проверьте ошибку RMS и примените результат."
        return candidate

    def apply(self, target_path):
        with self._lock:
            if self.state != "review" or not self.candidate:
                raise ValueError("нет рассчитанной калибровки для применения")
            data = {k: self.candidate[k] for k in (
                "image_width", "image_height", "camera_matrix", "distortion_coefficients"
            )}
        target_path = os.path.abspath(target_path)
        os.makedirs(os.path.dirname(target_path), exist_ok=True)
        if os.path.exists(target_path):
            shutil.copy2(target_path, target_path + ".previous")
        tmp = target_path + ".tmp"
        with open(tmp, "w", encoding="utf-8", newline="\n") as stream:
            yaml.safe_dump(data, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, target_path)
        with self._lock:
            self.state = "applied"
            self.guidance = "Новая калибровка применена к камере."
        return data

    def status(self):
        with self._lock:
            result = {
                "state": self.state,
                "inner_cols": self.inner_cols,
                "inner_rows": self.inner_rows,
                "square_mm": self.square_mm,
                "required_frames": self.required_frames,
                "captured_frames": len(self.image_points),
                "board_detected": self.board_detected,
                "sharpness": self.sharpness,
                "coverage_percent": self.coverage,
                "guidance": self.guidance,
                "error": self.error,
                "candidate": dict(self.candidate) if self.candidate else None,
            }
        return result

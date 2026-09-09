"""
tag_calibration_wizard.py - Finite State Machine (FSM) for guided auto-centering and tag calibration.

Features:
  - Motion authority mutex (IDLE, MANUAL, ROUTE, TEST, CALIBRATION).
  - Safety leases with REST heartbeat timeout (0.5s period, 1.0s timeout).
  - Target loss watchdog (< 0.25s during active servoing).
  - Visual servoing to center tag in image with correct coordinate signs (+x forward, +y left).
  - Mechanical oscillation settling delay (>= 0.8s) and stationary verification (|v_i| < 0.005 m/s).
  - Multi-frame stationary pose accumulation (>= 30 frames over >= 1.0s) with outlier rejection.
  - Safe speed clamps (max linear 0.04 m/s, max angular 0.15 rad/s).
"""

import time
import math
import numpy as np
from enum import Enum
from typing import Dict, List, Any, Optional, Tuple

try:
    from .geometry_transforms import (
        normalize_angle,
        pose_to_matrix,
        matrix_to_pose,
        invert_transform,
        optical_to_ros_matrix,
        ros_to_optical_matrix
    )
    from .single_tag_pnp import solve_single_tag_ippe
except ImportError:
    from geometry_transforms import (
        normalize_angle,
        pose_to_matrix,
        matrix_to_pose,
        invert_transform,
        optical_to_ros_matrix,
        ros_to_optical_matrix
    )
    from single_tag_pnp import solve_single_tag_ippe


class MotionAuthorityMode(str, Enum):
    IDLE = "IDLE"
    MANUAL = "MANUAL"
    ROUTE = "ROUTE"
    TEST = "TEST"
    CALIBRATION = "CALIBRATION"


class MotionAuthorityManager:
    """
    Mutex lease manager for robot motion authority.
    Prevents conflicting autopilot, manual teleop, test drives, and calibration wizard
    from simultaneously publishing cmd_vel.
    """
    def __init__(self, default_lease_sec: float = 1.0):
        self._current_mode = MotionAuthorityMode.IDLE
        self._lease_expiry: float = 0.0
        self._default_lease_sec = default_lease_sec

    @property
    def current_mode(self) -> MotionAuthorityMode:
        if self._current_mode != MotionAuthorityMode.IDLE:
            if time.time() > self._lease_expiry:
                self._current_mode = MotionAuthorityMode.IDLE
        return self._current_mode

    def request_lease(self, mode: MotionAuthorityMode, duration_sec: Optional[float] = None) -> Tuple[bool, str]:
        now = time.time()
        dur = duration_sec if duration_sec is not None else self._default_lease_sec
        active = self.current_mode

        if active == MotionAuthorityMode.IDLE or active == mode:
            self._current_mode = mode
            self._lease_expiry = now + dur
            return True, f"Lease granted for {mode.value} until {self._lease_expiry:.3f}"
        else:
            return False, f"Motion authority conflict: currently held by {active.value}"

    def renew_lease(self, mode: MotionAuthorityMode, duration_sec: Optional[float] = None) -> Tuple[bool, str]:
        now = time.time()
        dur = duration_sec if duration_sec is not None else self._default_lease_sec
        if self._current_mode == mode:
            self._lease_expiry = now + dur
            return True, f"Lease renewed for {mode.value}"
        return False, f"Cannot renew: active mode is {self.current_mode.value}, not {mode.value}"

    def release_lease(self, mode: MotionAuthorityMode) -> bool:
        if self._current_mode == mode:
            self._current_mode = MotionAuthorityMode.IDLE
            self._lease_expiry = 0.0
            return True
        return False


class WizardState(str, Enum):
    IDLE = "IDLE"
    REQUESTED = "REQUESTED"
    ACQUIRING_LEASES = "ACQUIRING_LEASES"
    ROUGH_SEEK = "ROUGH_SEEK"
    FINE_CENTERING = "FINE_CENTERING"
    SETTLING = "SETTLING"
    STOPPED_CHECK = "STOPPED_CHECK"
    STATIONARY_SOLVE = "STATIONARY_SOLVE"
    VERIFYING = "VERIFYING"
    COMPLETED = "COMPLETED"
    ABORTED = "ABORTED"


class TagCalibrationWizard:
    """
    Finite State Machine orchestrating tag calibration and auto-centering.
    """
    def __init__(self,
                 motion_manager: Optional[MotionAuthorityManager] = None,
                 cx: float = 320.0,
                 cy: float = 240.0,
                 centering_tol_px: float = 25.0,
                 target_loss_timeout_sec: float = 0.25,
                 heartbeat_timeout_sec: float = 1.0,
                 settling_delay_sec: float = 0.8,
                 min_stationary_frames: int = 30,
                 stationary_duration_sec: float = 1.0,
                 max_lin_vel: float = 0.04,
                 max_ang_vel: float = 0.15):
        self.motion_mgr = motion_manager if motion_manager is not None else MotionAuthorityManager()
        self.cx = cx
        self.cy = cy
        self.centering_tol_px = centering_tol_px
        self.target_loss_timeout_sec = target_loss_timeout_sec
        self.heartbeat_timeout_sec = heartbeat_timeout_sec
        self.settling_delay_sec = settling_delay_sec
        self.min_stationary_frames = min_stationary_frames
        self.stationary_duration_sec = stationary_duration_sec
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel

        # Internal state
        self.state = WizardState.IDLE
        self.target_tag_id: Optional[int] = None
        self.marker_size_m: float = 0.100
        self.last_heartbeat_time: float = 0.0
        self.last_detection_time: float = 0.0
        self.state_enter_time: float = 0.0
        self.abort_reason: str = ""

        # Stationary frame collection
        self.collected_samples: List[Dict[str, Any]] = []
        self.calibrated_tag_result: Optional[Dict[str, Any]] = None

    def start(self, target_tag_id: int, marker_size_m: float = 0.100, now: Optional[float] = None) -> Tuple[bool, str]:
        t = now if now is not None else time.time()
        if self.state not in (WizardState.IDLE, WizardState.COMPLETED, WizardState.ABORTED):
            return False, f"Wizard already active in state {self.state.value}"

        self.target_tag_id = int(target_tag_id)
        self.marker_size_m = float(marker_size_m)
        self.last_heartbeat_time = t
        self.last_detection_time = t
        self.state_enter_time = t
        self.abort_reason = ""
        self.collected_samples.clear()
        self.calibrated_tag_result = None

        # Request motion authority
        ok, msg = self.motion_mgr.request_lease(MotionAuthorityMode.CALIBRATION, self.heartbeat_timeout_sec)
        if not ok:
            self.state = WizardState.ABORTED
            self.abort_reason = f"Motion authority lease rejected: {msg}"
            return False, self.abort_reason

        self.state = WizardState.ACQUIRING_LEASES
        return True, f"Wizard initiated for tag {target_tag_id}"

    def heartbeat(self, now: Optional[float] = None) -> Tuple[bool, str]:
        t = now if now is not None else time.time()
        if self.state in (WizardState.IDLE, WizardState.COMPLETED, WizardState.ABORTED):
            return False, f"Cannot heartbeat in inactive state {self.state.value}"

        self.last_heartbeat_time = t
        ok, msg = self.motion_mgr.renew_lease(MotionAuthorityMode.CALIBRATION, self.heartbeat_timeout_sec)
        return ok, msg

    def abort(self, reason: str):
        self.state = WizardState.ABORTED
        self.abort_reason = str(reason)
        self.motion_mgr.release_lease(MotionAuthorityMode.CALIBRATION)

    def update(self,
               detections: List[Dict[str, Any]],
               robot_wheels_stopped: bool,
               current_robot_pose: Tuple[float, float, float],
               camera_matrix: np.ndarray,
               dist_coeffs: np.ndarray,
               T_base_cam: np.ndarray,
               now: Optional[float] = None) -> Tuple[WizardState, Optional[Tuple[float, float, float]], str]:
        """
        Main FSM update tick.
        
        Returns:
          (state, cmd_vel_tuple or None, status_message)
          cmd_vel_tuple is (vx, vy, omega) in robot REP-103 frame (+x forward, +y left, +w CCW).
        """
        t = now if now is not None else time.time()

        # Check heartbeat timeout
        if self.state not in (WizardState.IDLE, WizardState.COMPLETED, WizardState.ABORTED):
            if (t - self.last_heartbeat_time) > self.heartbeat_timeout_sec:
                self.abort(f"Heartbeat lease expired ({t - self.last_heartbeat_time:.2f}s > {self.heartbeat_timeout_sec}s)")
                return self.state, (0.0, 0.0, 0.0), self.abort_reason

        # Find target detection
        target_det = None
        for d in detections:
            if int(d.get("tag_id", -1)) == self.target_tag_id:
                target_det = d
                break

        if target_det is not None:
            self.last_detection_time = t

        # State dispatch
        if self.state == WizardState.IDLE:
            return WizardState.IDLE, None, "Wizard idle"

        if self.state == WizardState.ABORTED:
            return WizardState.ABORTED, (0.0, 0.0, 0.0), f"Aborted: {self.abort_reason}"

        if self.state == WizardState.COMPLETED:
            return WizardState.COMPLETED, (0.0, 0.0, 0.0), "Calibration complete"

        if self.state == WizardState.ACQUIRING_LEASES:
            # Lease is confirmed, proceed to seek / center
            self._transition_to(WizardState.FINE_CENTERING, t)
            return self.state, (0.0, 0.0, 0.0), "Leases acquired. Starting visual centering."

        if self.state == WizardState.FINE_CENTERING:
            # Target loss watchdog
            if (t - self.last_detection_time) > self.target_loss_timeout_sec:
                self.abort(f"Target tag {self.target_tag_id} lost for {t - self.last_detection_time:.2f}s > {self.target_loss_timeout_sec}s")
                return self.state, (0.0, 0.0, 0.0), self.abort_reason

            if target_det is None:
                # Momentary absence within watchdog window: halt motion
                return self.state, (0.0, 0.0, 0.0), "Target tag not detected in current frame, holding..."

            # Calculate center of marker in 2D image
            corners = np.array(target_det.get("corners_px", []), dtype=np.float64).reshape(-1, 2)
            if len(corners) != 4:
                return self.state, (0.0, 0.0, 0.0), "Malformed corners"

            u_tag = float(np.mean(corners[:, 0]))
            v_tag = float(np.mean(corners[:, 1]))

            # Pixel errors from image center (cx, cy)
            du = u_tag - self.cx
            dv = v_tag - self.cy
            pixel_dist = math.sqrt(du**2 + dv**2)

            if pixel_dist <= self.centering_tol_px:
                # Centering target reached! Transition to SETTLING
                self._transition_to(WizardState.SETTLING, t)
                return self.state, (0.0, 0.0, 0.0), f"Centering error {pixel_dist:.1f}px <= {self.centering_tol_px}px. Entering settling delay."

            # Visual Servoing Velocity Calculation:
            # Camera optical looking up at ceiling:
            #   Image -v (UP) corresponds to robot forward (+x)
            #   Image -u (LEFT) corresponds to robot left (+y)
            # Therefore:
            #   vx proportional to (cy - v_tag) = -dv
            #   vy proportional to (cx - u_tag) = -du
            kp = 0.0006  # velocity gain m/s per pixel
            vx = max(-self.max_lin_vel, min(self.max_lin_vel, -dv * kp))
            vy = max(-self.max_lin_vel, min(self.max_lin_vel, -du * kp))
            omega = 0.0

            return self.state, (float(vx), float(vy), float(omega)), f"Centering: error={pixel_dist:.1f}px, cmd=({vx:.3f}, {vy:.3f})"

        if self.state == WizardState.SETTLING:
            # Command zero velocity and wait mechanical settling
            elapsed = t - self.state_enter_time
            if elapsed >= self.settling_delay_sec:
                self._transition_to(WizardState.STOPPED_CHECK, t)
                return self.state, (0.0, 0.0, 0.0), f"Settled for {elapsed:.2f}s. Checking stopped status."
            return self.state, (0.0, 0.0, 0.0), f"Settling mechanical oscillation: {elapsed:.2f}s / {self.settling_delay_sec}s"

        if self.state == WizardState.STOPPED_CHECK:
            if robot_wheels_stopped:
                self.collected_samples.clear()
                self._transition_to(WizardState.STATIONARY_SOLVE, t)
                return self.state, (0.0, 0.0, 0.0), "Robot stopped verified. Commencing stationary solve accumulation."
            else:
                # If still moving after settling + 1s, abort
                if (t - self.state_enter_time) > 1.5:
                    self.abort("Robot wheels failed to achieve stationary threshold (<0.005 m/s)")
                    return self.state, (0.0, 0.0, 0.0), self.abort_reason
                return self.state, (0.0, 0.0, 0.0), "Waiting for stationary wheel threshold..."

        if self.state == WizardState.STATIONARY_SOLVE:
            if not robot_wheels_stopped:
                self.abort("Robot moved during stationary solve phase!")
                return self.state, (0.0, 0.0, 0.0), self.abort_reason

            if target_det is not None and target_det.get("pose_valid", False):
                corners = np.array(target_det.get("corners_px", []), dtype=np.float64).reshape(4, 2)
                res_pnp = solve_single_tag_ippe(corners, self.marker_size_m, camera_matrix, dist_coeffs)
                if res_pnp.get("pose_valid", False):
                    # Compute tag pose in map frame using current robot pose
                    # T_map_base
                    x_r, y_r, yaw_r = current_robot_pose
                    T_map_base = pose_to_matrix(x_r, y_r, 0.0, 0.0, 0.0, yaw_r)
                    T_map_camRos = T_map_base @ T_base_cam
                    T_camRos_tag = res_pnp["T_cameraRos_tag"]
                    T_map_tag = T_map_camRos @ T_camRos_tag
                    x_t, y_t, z_t, r_t, p_t, yaw_t = matrix_to_pose(T_map_tag)

                    self.collected_samples.append({
                        "x": x_t, "y": y_t, "z": z_t,
                        "roll": r_t, "pitch": p_t, "yaw": yaw_t,
                        "reproj_err": res_pnp.get("reproj_err", 0.0),
                        "distance_m": res_pnp.get("distance_m", 0.0),
                        "viewing_angle_deg": res_pnp.get("viewing_angle_deg", 0.0)
                    })

            elapsed = t - self.state_enter_time
            if len(self.collected_samples) >= self.min_stationary_frames and elapsed >= self.stationary_duration_sec:
                self._transition_to(WizardState.VERIFYING, t)
            else:
                return self.state, (0.0, 0.0, 0.0), f"Accumulating samples: {len(self.collected_samples)}/{self.min_stationary_frames} ({elapsed:.1f}s)"

        if self.state == WizardState.VERIFYING:
            if not self.collected_samples:
                self.abort("No valid stationary samples collected")
                return self.state, (0.0, 0.0, 0.0), self.abort_reason

            xs = [s["x"] for s in self.collected_samples]
            ys = [s["y"] for s in self.collected_samples]
            zs = [s["z"] for s in self.collected_samples]
            yaws = [s["yaw"] for s in self.collected_samples]
            errs = [s["reproj_err"] for s in self.collected_samples]
            views = [s["viewing_angle_deg"] for s in self.collected_samples]

            med_x = float(np.median(xs))
            med_y = float(np.median(ys))
            med_z = float(np.median(zs))
            med_yaw = float(np.median(yaws))
            med_err = float(np.median(errs))
            med_view = float(np.median(views))

            # Verification criteria
            if med_err > 2.0:
                self.abort(f"High reprojection error in calibration: {med_err:.2f}px > 2.0px")
                return self.state, (0.0, 0.0, 0.0), self.abort_reason
            if med_view > 30.0:
                self.abort(f"Viewing angle too steep: {med_view:.1f}deg > 30.0deg")
                return self.state, (0.0, 0.0, 0.0), self.abort_reason

            self.calibrated_tag_result = {
                "tag_id": self.target_tag_id,
                "state": "provisional",
                "enabled": True,
                "marker_size_m": self.marker_size_m,
                "pose": {
                    "x": round(med_x, 4),
                    "y": round(med_y, 4),
                    "z": round(med_z, 4),
                    "roll": round(math.pi, 4),
                    "pitch": 0.0,
                    "yaw": round(normalize_angle(med_yaw), 4)
                },
                "diagnostics": {
                    "samples_count": len(self.collected_samples),
                    "reproj_rms_px": round(med_err, 3),
                    "viewing_angle_deg": round(med_view, 1),
                    "std_x_mm": round(float(np.std(xs) * 1000.0), 2),
                    "std_y_mm": round(float(np.std(ys) * 1000.0), 2)
                }
            }

            self._transition_to(WizardState.COMPLETED, t)
            self.motion_mgr.release_lease(MotionAuthorityMode.CALIBRATION)
            return self.state, (0.0, 0.0, 0.0), f"Calibration successful for tag {self.target_tag_id}"

        return self.state, (0.0, 0.0, 0.0), f"Unhandled state {self.state.value}"

    def _transition_to(self, new_state: WizardState, now: float):
        self.state = new_state
        self.state_enter_time = now
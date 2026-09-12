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
import threading
import numpy as np
from enum import Enum
from typing import Dict, List, Any, Optional, Tuple

try:
    from .geometry_transforms import (
        normalize_angle,
        circular_mean,
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
        circular_mean,
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
    ESTOP = "ESTOP"


class MotionAuthorityManager:
    """
    Mutex lease manager for robot motion authority.
    Prevents conflicting autopilot, manual teleop, test drives, and calibration wizard
    from simultaneously publishing cmd_vel.
    Supports ESTOP override with infinite hold until cleared.
    """
    def __init__(self, default_lease_sec: float = 1.0):
        self._lock = threading.RLock()
        self._current_mode = MotionAuthorityMode.IDLE
        self._lease_expiry: float = 0.0
        self._default_lease_sec = default_lease_sec

    @property
    def current_mode(self) -> MotionAuthorityMode:
        with self._lock:
            if self._current_mode == MotionAuthorityMode.ESTOP:
                return MotionAuthorityMode.ESTOP
            if self._current_mode != MotionAuthorityMode.IDLE:
                if time.monotonic() > self._lease_expiry:
                    self._current_mode = MotionAuthorityMode.IDLE
            return self._current_mode

    def request_lease(self, mode: MotionAuthorityMode, duration_sec: Optional[float] = None) -> Tuple[bool, str]:
        with self._lock:
            if mode == MotionAuthorityMode.ESTOP:
                self._current_mode = MotionAuthorityMode.ESTOP
                self._lease_expiry = float('inf')
                return True, "E-STOP engaged"
            if self._current_mode == MotionAuthorityMode.ESTOP:
                return False, "Motion authority blocked: E-STOP is active"
            now = time.monotonic()
            dur = duration_sec if duration_sec is not None else self._default_lease_sec
            active = self.current_mode
            if active == MotionAuthorityMode.IDLE or active == mode:
                self._current_mode = mode
                self._lease_expiry = now + dur
                return True, f"Lease granted for {mode.value}"
            return False, f"Motion authority conflict: currently held by {active.value}"

    def renew_lease(self, mode: MotionAuthorityMode, duration_sec: Optional[float] = None) -> Tuple[bool, str]:
        with self._lock:
            if self._current_mode == MotionAuthorityMode.ESTOP:
                return False, "Cannot renew: E-STOP is active"
            if self._current_mode == mode:
                self._lease_expiry = time.monotonic() + (duration_sec or self._default_lease_sec)
                return True, f"Lease renewed for {mode.value}"
            return False, f"Cannot renew: active mode is {self.current_mode.value}, not {mode.value}"

    def release_lease(self, mode: MotionAuthorityMode) -> bool:
        with self._lock:
            if self._current_mode == mode:
                self._current_mode = MotionAuthorityMode.IDLE
                self._lease_expiry = 0.0
                return True
            return False

    def clear_estop(self) -> bool:
        with self._lock:
            if self._current_mode == MotionAuthorityMode.ESTOP:
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
    REVIEW = "REVIEW"
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
                 max_ang_vel: float = 0.15,
                 auto_confirm: bool = False):
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
        self.auto_confirm = auto_confirm

        # Internal state
        self.state = WizardState.IDLE
        self.target_tag_id: Optional[int] = None
        self.marker_size_m: float = 0.100
        self.last_heartbeat_time: float = 0.0
        self.last_detection_time: float = 0.0
        self.state_enter_time: float = 0.0
        self.abort_reason: str = ""

        # Closed-loop centering diagnostics and divergence protection.
        self.centering_axis: Optional[str] = None
        self.centering_error_px: Optional[float] = None
        self.centering_axis_error_px: Optional[float] = None
        self.centering_command: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._axis_best_error_px: Optional[float] = None
        self._axis_last_progress_time: float = 0.0

        # Stationary frame collection
        self.collected_samples: List[Dict[str, Any]] = []
        self.calibrated_tag_result: Optional[Dict[str, Any]] = None

    def start(self, target_tag_id: int, marker_size_m: float = 0.100, now: Optional[float] = None) -> Tuple[bool, str]:
        t = now if now is not None else time.monotonic()
        if self.state not in (WizardState.IDLE, WizardState.COMPLETED, WizardState.ABORTED):
            return False, f"Wizard already active in state {self.state.value}"

        self.target_tag_id = int(target_tag_id)
        self.marker_size_m = float(marker_size_m)
        self.last_heartbeat_time = t
        self.last_detection_time = t
        self.state_enter_time = t
        self.abort_reason = ""
        self.centering_axis = None
        self.centering_error_px = None
        self.centering_axis_error_px = None
        self.centering_command = (0.0, 0.0, 0.0)
        self._axis_best_error_px = None
        self._axis_last_progress_time = t
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
        t = now if now is not None else time.monotonic()
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
        t = now if now is not None else time.monotonic()

        # Always use the calibrated optical centre. The old 320x240 defaults
        # are wrong for the 640x600 camera (current cy is about 293 px).
        try:
            if camera_matrix is not None and np.asarray(camera_matrix).shape == (3, 3):
                cx = float(camera_matrix[0, 2])
                cy = float(camera_matrix[1, 2])
                if math.isfinite(cx) and math.isfinite(cy) and cx > 1.0 and cy > 1.0:
                    self.cx, self.cy = cx, cy
        except (TypeError, ValueError, IndexError):
            pass

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
            self.centering_error_px = float(pixel_dist)

            # Check if 3D base-frame error is available
            T_c_tag = target_det.get("T_cameraRos_tag")
            use_3d = False
            e_fwd = 0.0
            e_str = 0.0
            if T_c_tag is not None:
                try:
                    T_base_tag = T_base_cam @ T_c_tag
                    e_fwd = float(T_base_tag[0, 3])
                    e_str = float(T_base_tag[1, 3])
                    use_3d = True
                except Exception:
                    use_3d = False

            centered_by_3d = use_3d and (abs(e_fwd) <= 0.015 and abs(e_str) <= 0.015)

            if centered_by_3d:
                # Centering target reached! Transition to SETTLING
                self._transition_to(WizardState.SETTLING, t)
                return self.state, (0.0, 0.0, 0.0), f"Base-to-tag planar error <= 15 mm ({pixel_dist:.1f}px). Entering settling delay."

            # Calculate centering velocity in REP-103 (+X forward, +Y left)
            if use_3d:
                kp = 0.40
                # The physical three-wheel base distorts low-speed diagonal
                # commands. Centre one image axis at a time so the observed
                # motion remains predictable and cannot steer away diagonally.
                if self.centering_axis is None:
                    self.centering_axis = "horizontal" if abs(du) >= abs(dv) else "vertical"
                    self._axis_best_error_px = None
                    self._axis_last_progress_time = t
                elif self.centering_axis == "horizontal" and abs(e_str) <= 0.015:
                    self.centering_axis = "vertical"
                    self._axis_best_error_px = None
                    self._axis_last_progress_time = t
                elif self.centering_axis == "vertical" and abs(e_fwd) <= 0.015:
                    self.centering_axis = "horizontal"
                    self._axis_best_error_px = None
                    self._axis_last_progress_time = t

                if self.centering_axis == "horizontal":
                    vx = 0.0
                    vy = max(-self.max_lin_vel, min(self.max_lin_vel, e_str * kp))
                    if abs(e_str) < 0.010:
                        vy = 0.0
                    axis_error_px = abs(du)
                else:
                    vx = max(-self.max_lin_vel, min(self.max_lin_vel, e_fwd * kp))
                    vy = 0.0
                    if abs(e_fwd) < 0.010:
                        vx = 0.0
                    axis_error_px = abs(dv)

                self.centering_axis_error_px = float(axis_error_px)
                if self._axis_best_error_px is None or axis_error_px < self._axis_best_error_px - 1.0:
                    self._axis_best_error_px = float(axis_error_px)
                    self._axis_last_progress_time = t
                elif (t - self._axis_last_progress_time) > 1.25 and axis_error_px > self._axis_best_error_px + 6.0:
                    self.abort(
                        f"Centering divergence on {self.centering_axis} axis: "
                        f"error grew from {self._axis_best_error_px:.1f}px to {axis_error_px:.1f}px"
                    )
                    self.centering_command = (0.0, 0.0, 0.0)
                    return self.state, self.centering_command, self.abort_reason

                if (t - self.state_enter_time) > 25.0:
                    self.abort(f"Centering timeout: target did not converge within 25.0s (error {pixel_dist:.1f}px)")
                    self.centering_command = (0.0, 0.0, 0.0)
                    return self.state, self.centering_command, self.abort_reason
            else:
                # Pixel directions depend on the measured camera mounting.
                # Moving on a guessed 2-D sign convention can drive away from
                # the tag, so calibration requires the validated 3-D pose.
                return self.state, (0.0, 0.0, 0.0), "Target has no valid 3-D pose; holding"

            omega = 0.0
            self.centering_command = (float(vx), float(vy), float(omega))
            return self.state, self.centering_command, (
                f"Centering {self.centering_axis}: error={pixel_dist:.1f}px, "
                f"cmd=({vx:.3f}, {vy:.3f})"
            )

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
            pitches = [s.get("pitch", 0.0) for s in self.collected_samples]

            # Outlier rejection: 3-sigma from median
            med_x = float(np.median(xs))
            med_y = float(np.median(ys))
            med_z = float(np.median(zs))
            std_x = max(1e-4, float(np.std(xs)))
            std_y = max(1e-4, float(np.std(ys)))
            std_z = max(1e-4, float(np.std(zs)))

            inlier_samples = [
                s for s in self.collected_samples
                if abs(s["x"] - med_x) <= 3.0 * std_x
                and abs(s["y"] - med_y) <= 3.0 * std_y
                and abs(s["z"] - med_z) <= 3.0 * std_z
            ]
            if len(inlier_samples) < 3:
                inlier_samples = self.collected_samples

            i_xs = [s["x"] for s in inlier_samples]
            i_ys = [s["y"] for s in inlier_samples]
            i_zs = [s["z"] for s in inlier_samples]
            i_yaws = [s["yaw"] for s in inlier_samples]
            i_errs = [s["reproj_err"] for s in inlier_samples]
            i_views = [s["viewing_angle_deg"] for s in inlier_samples]
            i_pitches = [s.get("pitch", 0.0) for s in inlier_samples]

            final_x = float(np.mean(i_xs))
            final_y = float(np.mean(i_ys))
            final_z = float(np.mean(i_zs))
            final_yaw = circular_mean(i_yaws)
            yaw_residuals = [normalize_angle(y - final_yaw) for y in i_yaws]
            final_err = float(np.mean(i_errs))
            final_view = float(np.mean(i_views))
            med_pitch = float(np.median(i_pitches))

            # Quality criteria
            if final_err > 2.0:
                self.abort(f"High reprojection error in calibration: {final_err:.2f}px > 2.0px")
                return self.state, (0.0, 0.0, 0.0), self.abort_reason
            if final_view > 35.0:
                self.abort(f"Viewing angle too steep: {final_view:.1f}deg > 35.0deg")
                return self.state, (0.0, 0.0, 0.0), self.abort_reason
            # Ceiling plane check: pitch within +/- 5 deg (0.09 rad)
            if abs(med_pitch) > 0.09:
                self.abort(f"Tag out-of-plane pitch: {math.degrees(med_pitch):.1f}deg > 5.0deg")
                return self.state, (0.0, 0.0, 0.0), self.abort_reason

            cov_3x3 = [
                [round(float(np.var(i_xs)), 6), 0.0, 0.0],
                [0.0, round(float(np.var(i_ys)), 6), 0.0],
                [0.0, 0.0, round(float(np.var(yaw_residuals)), 6)]
            ]

            self.calibrated_tag_result = {
                "tag_id": self.target_tag_id,
                "state": "provisional",
                "enabled": True,
                "marker_size_m": self.marker_size_m,
                "pose": {
                    "x": round(final_x, 4),
                    "y": round(final_y, 4),
                    "z": round(final_z, 4),
                    "roll": round(math.pi, 4),
                    "pitch": 0.0,
                    "yaw": round(normalize_angle(final_yaw), 4)
                },
                "covariance": cov_3x3,
                "diagnostics": {
                    "samples_count": len(self.collected_samples),
                    "inliers_count": len(inlier_samples),
                    "reproj_rms_px": round(final_err, 3),
                    "viewing_angle_deg": round(final_view, 1),
                    "std_x_mm": round(float(np.std(i_xs) * 1000.0), 2),
                    "std_y_mm": round(float(np.std(i_ys) * 1000.0), 2)
                }
            }

            if getattr(self, 'auto_confirm', False):
                self._transition_to(WizardState.COMPLETED, t)
                self.motion_mgr.release_lease(MotionAuthorityMode.CALIBRATION)
                return self.state, (0.0, 0.0, 0.0), f"Calibration successful for tag {self.target_tag_id}"
            else:
                self._transition_to(WizardState.REVIEW, t)
                return self.state, (0.0, 0.0, 0.0), f"Calibration complete, awaiting user review for tag {self.target_tag_id}"

        if self.state == WizardState.REVIEW:
            return self.state, (0.0, 0.0, 0.0), "Awaiting user confirmation in REVIEW state"

        return self.state, (0.0, 0.0, 0.0), f"Unhandled state {self.state.value}"

    def confirm_review(self) -> Tuple[bool, str]:
        """Confirm calibration result from REVIEW state and transition to COMPLETED."""
        if self.state != WizardState.REVIEW:
            return False, f"Cannot confirm calibration in state {self.state.value}"
        self._transition_to(WizardState.COMPLETED, time.monotonic())
        self.motion_mgr.release_lease(MotionAuthorityMode.CALIBRATION)
        return True, "Calibration result confirmed by user"

    def _transition_to(self, new_state: WizardState, now: float):
        self.state = new_state
        self.state_enter_time = now

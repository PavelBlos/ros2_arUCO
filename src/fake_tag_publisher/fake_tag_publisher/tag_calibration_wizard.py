"""
tag_calibration_wizard.py - Finite State Machine (FSM) for guided auto-centering and tag calibration.

Features:
  - Motion authority mutex (IDLE, MANUAL, ROUTE, TEST, CALIBRATION).
  - Safety leases with REST heartbeat timeout (0.5s period, 1.0s timeout).
  - Target loss watchdog (< 0.25s during active servoing).
  - Visual servoing to the geometric image centre (+x forward, +y left).
  - Mechanical oscillation settling delay (>= 0.8s) and stationary verification (|v_i| < 0.005 m/s).
  - Multi-frame stationary pose accumulation (>= 30 frames over >= 1.0s) with outlier rejection.
  - Safe speed clamps (max linear 0.04 m/s, max angular 0.15 rad/s).
"""

import time
import math
import threading
import numpy as np
try:
    from .ceiling_geometry import ceiling_rays, locate_ceiling_tag, solve_ceiling_frame, vertical_target_pixel
except ImportError:
    from ceiling_geometry import ceiling_rays, locate_ceiling_tag, solve_ceiling_frame, vertical_target_pixel
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
                 centering_tol_px: float = 6.0,
                 target_loss_timeout_sec: float = 0.45,
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
        self.ceiling_planar = False
        self.ceiling_height_m = 2.5
        self._last_sample_stamp = None

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
        self.centering_du_px: Optional[float] = None
        self.centering_dv_px: Optional[float] = None
        self.centering_forward_error_m: Optional[float] = None
        self.centering_strafe_error_m: Optional[float] = None
        self.centering_direction_corrections: int = 0
        self.centering_phase: str = "idle"
        self.centering_response_matrix: Optional[List[List[float]]] = None
        self.centering_trace: List[Dict[str, Any]] = []
        self._best_total_error_px: Optional[float] = None
        self._last_total_progress_time: float = 0.0
        self._centered_since: Optional[float] = None
        # The camera can be rotated relative to the robot. Measure the full
        # 2x2 mapping from body translation to image motion at the start of a
        # run instead of assuming that image and body axes are parallel.
        self._response_matrix: Optional[np.ndarray] = None
        self._cached_response_matrix: Optional[np.ndarray] = None
        self._probe_start_uv: Optional[np.ndarray] = None
        self._probe_forward_delta: Optional[np.ndarray] = None
        self._probe_phase_start: float = 0.0
        self._probe_speed: float = min(0.030, self.max_lin_vel)
        self._probe_motion_sec: float = 0.65
        self._probe_settle_sec: float = 0.35
        self._probe_retry_limit: int = 1
        self._probe_forward_retries: int = 0
        self._probe_strafe_retries: int = 0

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
        self.centering_du_px = None
        self.centering_dv_px = None
        self.centering_forward_error_m = None
        self.centering_strafe_error_m = None
        self.centering_direction_corrections = 0
        cached = self._cached_response_matrix
        if cached is not None and np.all(np.isfinite(cached)) and abs(float(np.linalg.det(cached))) >= 0.25:
            self._response_matrix = cached.copy()
            self.centering_response_matrix = cached.tolist()
            self.centering_phase = "servo"
        else:
            self._response_matrix = None
            self.centering_response_matrix = None
            self.centering_phase = "probe_forward"
        self.centering_trace.clear()
        self._best_total_error_px = None
        self._last_total_progress_time = t
        self._centered_since = None
        self._probe_start_uv = None
        self._probe_forward_delta = None
        self._probe_phase_start = t
        self._probe_forward_retries = 0
        self._probe_strafe_retries = 0
        self.collected_samples.clear()
        self._last_sample_stamp = None
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
               known_tags: Optional[Dict[str, Any]] = None,
               now: Optional[float] = None) -> Tuple[WizardState, Optional[Tuple[float, float, float]], str]:
        """
        Main FSM update tick.
        
        Returns:
          (state, cmd_vel_tuple or None, status_message)
          cmd_vel_tuple is (vx, vy, omega) in robot REP-103 frame (+x forward, +y left, +w CCW).
        """
        t = now if now is not None else time.monotonic()

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
            self.centering_du_px = float(du)
            self.centering_dv_px = float(dv)

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

            self.centering_forward_error_m = float(e_fwd) if use_3d else None
            self.centering_strafe_error_m = float(e_str) if use_3d else None

            if self.ceiling_planar:
                try:
                    rays = ceiling_rays(corners, camera_matrix, dist_coeffs, T_base_cam)
                    height = self.ceiling_height_m - T_base_cam[2, 3]
                    error_body = height * rays.mean(axis=0) + T_base_cam[:2, 3]
                    e_fwd, e_str = error_body
                    self.centering_forward_error_m = float(e_fwd)
                    self.centering_strafe_error_m = float(e_str)
                    use_3d = True
                except (ValueError, np.linalg.LinAlgError):
                    return self.state, (0., 0., 0.), 'Ceiling geometry invalid; holding'

            # The camera overlay and the user both refer to the geometric
            # frame centre. Do not mix this pixel objective with a separate
            # 3-D threshold: that previously left one axis driving for 25 s
            # even after its pixel error had already reached the target.
            centered_by_pixel = abs(du) <= self.centering_tol_px and abs(dv) <= self.centering_tol_px
            if centered_by_pixel:
                if self._centered_since is None:
                    self._centered_since = t
                self.centering_axis = None
                self.centering_axis_error_px = max(abs(du), abs(dv))
                self.centering_command = (0.0, 0.0, 0.0)
                if (t - self._centered_since) >= 0.35:
                    self._transition_to(WizardState.SETTLING, t)
                    return self.state, self.centering_command, (
                        f"Target held inside +/-{self.centering_tol_px:.0f}px for 0.35s. Entering settling delay."
                    )
                return self.state, self.centering_command, "Target is centered; verifying stability..."
            self._centered_since = None

            if self.ceiling_planar:
                if self._best_total_error_px is None or pixel_dist < self._best_total_error_px - 1.0:
                    self._best_total_error_px = pixel_dist
                    self._last_total_progress_time = t
                if t-self.state_enter_time > 35.0 or t-self._last_total_progress_time > 5.0:
                    self.abort('Centering did not converge with calibrated camera/body geometry')
                    return self.state, (0., 0., 0.), self.abort_reason
                velocity = .6 * np.array([e_fwd, e_str])
                velocity *= min(1., self.max_lin_vel/max(1e-9, np.linalg.norm(velocity)))
                self.centering_phase = 'calibrated_geometry'
                self.centering_axis = 'combined'
                self.centering_command = (float(velocity[0]), float(velocity[1]), 0.)
                self._append_trace(t, u_tag, v_tag, du, dv, self.centering_command)
                return self.state, self.centering_command, 'Centering above base origin with calibrated camera geometry'

            # Calculate centering velocity in REP-103 (+X forward, +Y left).
            if use_3d:
                if self._response_matrix is None:
                    cmd, probe_message = self._update_response_probe(u_tag, v_tag, t)
                    self.centering_command = cmd
                    self.centering_axis = self.centering_phase
                    self.centering_axis_error_px = max(abs(du), abs(dv))
                    self._append_trace(t, u_tag, v_tag, du, dv, cmd)
                    if self.state == WizardState.ABORTED:
                        return self.state, (0.0, 0.0, 0.0), self.abort_reason
                    return self.state, cmd, probe_message

                self.centering_phase = "servo"
                self.centering_axis = "combined"
                self.centering_axis_error_px = max(abs(du), abs(dv))
                try:
                    # Columns describe image motion caused by positive robot
                    # forward and positive robot-left motion. Solve for the
                    # body direction which moves the observed error to zero.
                    body_coeff = np.linalg.solve(
                        self._response_matrix,
                        np.array([-du, -dv], dtype=np.float64)
                    )
                except np.linalg.LinAlgError:
                    self.abort("Measured camera/body response matrix became singular")
                    return self.state, (0.0, 0.0, 0.0), self.abort_reason

                max_coeff = float(np.max(np.abs(body_coeff)))
                if not math.isfinite(max_coeff) or max_coeff < 1e-9:
                    self.abort("Invalid visual-servo command from measured response")
                    return self.state, (0.0, 0.0, 0.0), self.abort_reason
                body_direction = body_coeff / max_coeff
                # Slow down continuously near the cross. The former 0.018 m/s
                # floor made the robot enter an 18 px window with too much
                # residual motion and stop visibly short of the exact centre.
                speed = min(self.max_lin_vel, max(0.008, pixel_dist * 0.00065))
                vx = float(body_direction[0] * speed)
                vy = float(body_direction[1] * speed)

                if self._best_total_error_px is None or pixel_dist < self._best_total_error_px - 1.5:
                    self._best_total_error_px = float(pixel_dist)
                    self._last_total_progress_time = t
                elif ((t - self._last_total_progress_time) > 4.0
                      and pixel_dist > self._best_total_error_px + 10.0):
                    self.abort(
                        f"Centering is not converging: total error grew from "
                        f"{self._best_total_error_px:.1f}px to {pixel_dist:.1f}px"
                    )
                    self.centering_command = (0.0, 0.0, 0.0)
                    return self.state, self.centering_command, self.abort_reason

                if (t - self.state_enter_time) > 35.0:
                    self.abort(f"Centering timeout: target did not converge within 35.0s (error {pixel_dist:.1f}px)")
                    self.centering_command = (0.0, 0.0, 0.0)
                    return self.state, self.centering_command, self.abort_reason
            else:
                # Pixel directions depend on the measured camera mounting.
                # Moving on a guessed 2-D sign convention can drive away from
                # the tag, so calibration requires the validated 3-D pose.
                return self.state, (0.0, 0.0, 0.0), "Target has no valid 3-D pose; holding"

            omega = 0.0
            self.centering_command = (float(vx), float(vy), float(omega))
            self._append_trace(t, u_tag, v_tag, du, dv, self.centering_command)
            return self.state, self.centering_command, (
                f"Centering with measured 2-D response: error={pixel_dist:.1f}px, "
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

            if self.ceiling_planar and target_det is not None:
                stamp = target_det.get('frame_stamp', t)
                if stamp != self._last_sample_stamp:
                    references = {k:v for k,v in (known_tags or {}).items() if str(k) != str(self.target_tag_id)}
                    ref_pose = solve_ceiling_frame(detections, references, camera_matrix, dist_coeffs, T_base_cam)
                    pose = ref_pose.get('fused_base_pose')
                    visible_reference_ids = {
                        str(d.get('tag_id')) for d in detections
                        if str(d.get('tag_id')) in references
                    }
                    if pose is None and visible_reference_ids:
                        if (t - self.state_enter_time) > 4.0:
                            self.abort('Visible reference tag is inconsistent with the ceiling map')
                            return self.state, (0., 0., 0.), self.abort_reason
                        return self.state, (0., 0., 0.), 'Waiting for a consistent independent reference tag'
                    used_reference = pose is not None
                    pose = current_robot_pose if pose is None else pose
                    mapped = locate_ceiling_tag(target_det, self.marker_size_m, pose, camera_matrix, dist_coeffs, T_base_cam)
                    # This profile has one physical ceiling plane. Apparent-size
                    # noise must not create a different map Z for every tag.
                    mapped['z'] = float(self.ceiling_height_m)
                    self.collected_samples.append({**mapped, 'reproj_err':float(target_det.get('reproj_err', 0.)),
                        'distance_m':mapped['z'], 'viewing_angle_deg':0.,
                        'mapping_source':'covisible_tag' if used_reference else 'odometry_fallback'})
                    self._last_sample_stamp = stamp
            elif target_det is not None and target_det.get("pose_valid", False):
                corners = np.array(target_det.get("corners_px", []), dtype=np.float64).reshape(4, 2)
                res_pnp = solve_single_tag_ippe(corners, self.marker_size_m, camera_matrix, dist_coeffs)
                if res_pnp.get("pose_valid", False):
                    T_camRos_tag = res_pnp["T_cameraRos_tag"]
                    # A simultaneously visible known marker gives a direct
                    # map-to-target transform.  This cancels camera mounting
                    # error and avoids projecting the camera's 6-DoF pose onto
                    # the robot's planar SE(2) pose before locating the tag.
                    T_map_tag, reference_id = self._map_tag_from_covisible_reference(
                        T_camRos_tag, detections, known_tags or {}
                    )
                    mapping_source = "covisible_tag"
                    if T_map_tag is None:
                        x_r, y_r, yaw_r = current_robot_pose
                        T_map_base = pose_to_matrix(x_r, y_r, 0.0, 0.0, 0.0, yaw_r)
                        T_map_camRos = T_map_base @ T_base_cam
                        T_map_tag = T_map_camRos @ T_camRos_tag
                        reference_id = None
                        mapping_source = "odometry_fallback"
                    x_t, y_t, z_t, r_t, p_t, yaw_t = matrix_to_pose(T_map_tag)

                    self.collected_samples.append({
                        "x": x_t, "y": y_t, "z": z_t,
                        "roll": r_t, "pitch": p_t, "yaw": yaw_t,
                        "mapping_source": mapping_source,
                        "reference_tag_id": reference_id,
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
            i_rolls = [s.get("roll", math.pi) for s in inlier_samples]
            i_yaws = [s["yaw"] for s in inlier_samples]
            i_errs = [s["reproj_err"] for s in inlier_samples]
            i_views = [s["viewing_angle_deg"] for s in inlier_samples]
            i_pitches = [s.get("pitch", 0.0) for s in inlier_samples]

            final_x = float(np.mean(i_xs))
            final_y = float(np.mean(i_ys))
            final_z = float(np.mean(i_zs))
            final_roll = circular_mean(i_rolls)
            final_yaw = circular_mean(i_yaws)
            yaw_residuals = [normalize_angle(y - final_yaw) for y in i_yaws]
            final_err = float(np.mean(i_errs))
            final_view = float(np.mean(i_views))
            med_pitch = float(np.median(i_pitches))
            mapping_sources = [s.get("mapping_source", "odometry_fallback") for s in inlier_samples]
            covisible_count = sum(source == "covisible_tag" for source in mapping_sources)

            # Quality criteria
            if final_err > 2.0:
                self.abort(f"High reprojection error in calibration: {final_err:.2f}px > 2.0px")
                return self.state, (0.0, 0.0, 0.0), self.abort_reason
            if final_view > 35.0:
                self.abort(f"Viewing angle too steep: {final_view:.1f}deg > 35.0deg")
                return self.state, (0.0, 0.0, 0.0), self.abort_reason
            pitch_deg = math.degrees(med_pitch)
            # IPPE pitch around 5-8 degrees is normal with small printed tags
            # and a wide-angle view. Reject only a clearly non-planar target;
            # retain a warning for review above the preferred 5 degree band.
            if not self.ceiling_planar and abs(pitch_deg) > 15.0:
                self.abort(f"Tag out-of-plane pitch: {pitch_deg:.1f}deg > 15.0deg")
                return self.state, (0.0, 0.0, 0.0), self.abort_reason
            quality_warnings = []
            if not self.ceiling_planar and abs(pitch_deg) > 5.0:
                quality_warnings.append(
                    f"Measured tag pitch {pitch_deg:.1f}deg is outside the preferred +/-5deg band"
                )

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
                    "roll": round(normalize_angle(final_roll), 4),
                    "pitch": round(med_pitch, 4),
                    "yaw": round(normalize_angle(final_yaw), 4)
                },
                "covariance": cov_3x3,
                "diagnostics": {
                    "samples_count": len(self.collected_samples),
                    "inliers_count": len(inlier_samples),
                    "reproj_rms_px": round(final_err, 3),
                    "viewing_angle_deg": round(final_view, 1),
                    "tag_pitch_deg": round(pitch_deg, 1),
                    "warnings": quality_warnings,
                    "std_x_mm": round(float(np.std(i_xs) * 1000.0), 2),
                    "std_y_mm": round(float(np.std(i_ys) * 1000.0), 2),
                    "mapping_source": "covisible_tag" if covisible_count else "odometry_fallback",
                    "covisible_samples": covisible_count
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

    def _map_tag_from_covisible_reference(self, T_camera_tag, detections, known_tags):
        """Locate a target from the best known marker visible in the same frame."""
        candidates = []
        for det in detections:
            ref_id = str(det.get("tag_id"))
            if ref_id == str(self.target_tag_id) or not det.get("pose_valid", False):
                continue
            ref_info = known_tags.get(ref_id)
            T_camera_ref = det.get("T_cameraRos_tag")
            if not ref_info or T_camera_ref is None:
                continue
            pose = ref_info.get("pose", {})
            try:
                T_map_ref = pose_to_matrix(
                    float(pose["x"]), float(pose["y"]), float(pose["z"]),
                    float(pose["roll"]), float(pose["pitch"]), float(pose["yaw"])
                )
                T_map_target = (
                    T_map_ref
                    @ invert_transform(np.asarray(T_camera_ref))
                    @ np.asarray(T_camera_tag)
                )
                quality = float(det.get("reproj_err", 999.0))
                candidates.append((quality, int(ref_id), T_map_target))
            except (KeyError, TypeError, ValueError, np.linalg.LinAlgError):
                continue
        if not candidates:
            return None, None
        _, reference_id, result = min(candidates, key=lambda item: item[0])
        return result, reference_id

    def _update_response_probe(self, u_tag: float, v_tag: float, now: float):
        """Measure image response to two short orthogonal robot motions."""
        uv = np.array([u_tag, v_tag], dtype=np.float64)
        zero = (0.0, 0.0, 0.0)

        if self.centering_phase == "probe_forward":
            if self._probe_start_uv is None:
                self._probe_start_uv = uv.copy()
                self._probe_phase_start = now
            probe_duration = self._probe_motion_sec + 0.35 * self._probe_forward_retries
            if (now - self._probe_phase_start) < probe_duration:
                return (self._probe_speed, 0.0, 0.0), "Measuring camera response: short forward motion"
            self.centering_phase = "probe_forward_settle"
            self._probe_phase_start = now
            return zero, "Forward probe complete; waiting for image to settle"

        if self.centering_phase == "probe_forward_settle":
            if (now - self._probe_phase_start) < self._probe_settle_sec:
                return zero, "Waiting after forward response probe"
            delta = uv - self._probe_start_uv
            if float(np.linalg.norm(delta)) < 3.0:
                if self._probe_forward_retries < self._probe_retry_limit:
                    self._probe_forward_retries += 1
                    self._probe_start_uv = uv.copy()
                    self._probe_phase_start = now
                    self.centering_phase = "probe_forward"
                    return zero, "Forward response was too small; retrying with a longer probe"
                self.abort("Forward response probe stayed below 3 px after retry")
                return zero, self.abort_reason
            self._probe_forward_delta = delta
            self._probe_start_uv = uv.copy()
            self._probe_phase_start = now
            self.centering_phase = "probe_strafe"
            return zero, "Forward response measured; starting left-motion probe"

        if self.centering_phase == "probe_strafe":
            probe_duration = self._probe_motion_sec + 0.35 * self._probe_strafe_retries
            if (now - self._probe_phase_start) < probe_duration:
                return (0.0, self._probe_speed, 0.0), "Measuring camera response: short left motion"
            self.centering_phase = "probe_strafe_settle"
            self._probe_phase_start = now
            return zero, "Left-motion probe complete; waiting for image to settle"

        if self.centering_phase == "probe_strafe_settle":
            if (now - self._probe_phase_start) < self._probe_settle_sec:
                return zero, "Waiting after left-motion response probe"
            strafe_delta = uv - self._probe_start_uv
            if float(np.linalg.norm(strafe_delta)) < 3.0:
                if self._probe_strafe_retries < self._probe_retry_limit:
                    self._probe_strafe_retries += 1
                    self._probe_start_uv = uv.copy()
                    self._probe_phase_start = now
                    self.centering_phase = "probe_strafe"
                    return zero, "Left-motion response was too small; retrying with a longer probe"
                self.abort("Left-motion response probe stayed below 3 px after retry")
                return zero, self.abort_reason
            forward_unit = self._probe_forward_delta / np.linalg.norm(self._probe_forward_delta)
            strafe_unit = strafe_delta / np.linalg.norm(strafe_delta)
            response = np.column_stack((forward_unit, strafe_unit))
            determinant = float(np.linalg.det(response))
            if not math.isfinite(determinant) or abs(determinant) < 0.25:
                self.abort(
                    f"Camera/body response probes are not independent (det={determinant:.2f})"
                )
                return zero, self.abort_reason
            self._response_matrix = response
            self._cached_response_matrix = response.copy()
            self.centering_response_matrix = response.tolist()
            self.centering_phase = "servo"
            self._best_total_error_px = None
            self._last_total_progress_time = now
            return zero, "Camera/body response measured; starting closed-loop centering"

        self.abort(f"Unknown centering phase: {self.centering_phase}")
        return zero, self.abort_reason

    def _append_trace(self, now, u_tag, v_tag, du, dv, command):
        self.centering_trace.append({
            "t": round(float(now - self.state_enter_time), 3),
            "phase": self.centering_phase,
            "u": round(float(u_tag), 2),
            "v": round(float(v_tag), 2),
            "du": round(float(du), 2),
            "dv": round(float(dv), 2),
            "command": [round(float(value), 4) for value in command],
        })
        if len(self.centering_trace) > 250:
            del self.centering_trace[:-250]

    def _transition_to(self, new_state: WizardState, now: float):
        self.state = new_state
        self.state_enter_time = now

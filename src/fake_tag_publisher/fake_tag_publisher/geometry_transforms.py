"""
geometry_transforms.py - Rigorous SE(3) and SE(2) Coordinate Transformation Library.

Conventions:
  - T_A_B transforms a point represented in frame B into frame A: p_A = T_A_B @ p_B.
  - Optical Frame: X right, Y down, Z forward.
  - ROS REP-103 Frame: X forward, Y left, Z up.
  - SE(2) represents planar pose (x, y, yaw).
"""

import math
import numpy as np
from scipy.spatial.transform import Rotation as R

# ---------------------------------------------------------------------------
# Angle and Clock Helpers
# ---------------------------------------------------------------------------

def normalize_angle(angle: float) -> float:
    """Normalize angle in radians to [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))

def map_velocity_to_body(v_map_x: float, v_map_y: float, body_yaw: float):
    """Rotate map velocity into REP-103 body axes as (forward, left)."""
    cos_yaw = math.cos(body_yaw)
    sin_yaw = math.sin(body_yaw)
    forward = v_map_x * cos_yaw + v_map_y * sin_yaw
    left = -v_map_x * sin_yaw + v_map_y * cos_yaw
    return float(forward), float(left)

def circular_mean(angles, weights=None) -> float:
    """Compute weighted circular mean of angles in radians."""
    angles = np.asarray(angles, dtype=np.float64)
    if weights is None:
        weights = np.ones_like(angles, dtype=np.float64)
    else:
        weights = np.asarray(weights, dtype=np.float64)
    
    sin_sum = np.sum(weights * np.sin(angles))
    cos_sum = np.sum(weights * np.cos(angles))
    return float(math.atan2(sin_sum, cos_sum))

def compute_midpoint_stamp(t_before: float, t_after: float) -> float:
    """Compute the midpoint timestamp between blocking read start and end."""
    return (t_before + t_after) / 2.0

def compute_latency_ms(capture_stamp: float, current_time: float = None) -> float:
    """Compute elapsed processing latency in milliseconds."""
    if current_time is None:
        import time
        current_time = time.time()
    return max(0.0, (current_time - capture_stamp) * 1000.0)

# ---------------------------------------------------------------------------
# Optical and ROS Frame Conventions
# ---------------------------------------------------------------------------

def optical_to_ros_rotation() -> np.ndarray:
    """
    Rotation matrix converting from OpenCV optical frame to ROS REP-103 frame:
      X_ros = Z_opt (forward)
      Y_ros = -X_opt (left)
      Z_ros = -Y_opt (up)
    """
    return np.array([
        [0.0,  0.0,  1.0],
        [-1.0, 0.0,  0.0],
        [0.0, -1.0,  0.0]
    ], dtype=np.float64)

def optical_to_ros_matrix() -> np.ndarray:
    """4x4 SE(3) matrix transforming vectors from OpenCV optical to ROS frame."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = optical_to_ros_rotation()
    return T

def ros_to_optical_matrix() -> np.ndarray:
    """4x4 SE(3) matrix transforming vectors from ROS frame to OpenCV optical."""
    return invert_transform(optical_to_ros_matrix())

# ---------------------------------------------------------------------------
# SE(3) Transformations
# ---------------------------------------------------------------------------

def pose_to_matrix(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Construct 4x4 SE(3) transformation matrix from position and Euler angles (XYZ order)."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T

def matrix_to_pose(T: np.ndarray):
    """Extract (x, y, z, roll, pitch, yaw) from a 4x4 SE(3) transformation matrix."""
    x, y, z = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])
    rpy = R.from_matrix(T[:3, :3]).as_euler('xyz')
    return x, y, z, float(rpy[0]), float(rpy[1]), float(rpy[2])

def invert_transform(T: np.ndarray) -> np.ndarray:
    """
    Exact SE(3) inversion using orthogonality of R:
      T = [ R  t ]  =>  T^-1 = [ R^T  -R^T @ t ]
          [ 0  1 ]             [  0       1     ]
    """
    R_mat = T[:3, :3]
    t_vec = T[:3, 3]
    R_inv = R_mat.T
    t_inv = -R_inv @ t_vec
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    return T_inv

def camera_pose_from_tag(T_map_tag: np.ndarray, T_cam_tag: np.ndarray) -> np.ndarray:
    """
    Compute camera pose in map frame:
      T_map_cam = T_map_tag @ inverse(T_cam_tag)
    """
    return T_map_tag @ invert_transform(T_cam_tag)

def base_pose_from_camera(T_map_cam: np.ndarray, T_base_cam: np.ndarray) -> np.ndarray:
    """
    Compute base_link pose in map frame:
      T_map_base = T_map_cam @ inverse(T_base_cam)
    """
    return T_map_cam @ invert_transform(T_base_cam)

# ---------------------------------------------------------------------------
# Exact SE(2) Map -> Odom Composition
# ---------------------------------------------------------------------------

def compute_map_to_odom_se2(x_mb: float, y_mb: float, yaw_mb: float,
                            x_ob: float, y_ob: float, yaw_ob: float):
    """
    Compute exact SE(2) transformation T_map_odom such that:
      T_map_base = T_map_odom @ T_odom_base
      => T_map_odom = T_map_base @ inverse(T_odom_base)

    Args:
      x_mb, y_mb, yaw_mb: Robot base pose in map frame (visual observation)
      x_ob, y_ob, yaw_ob: Robot base pose in odom frame (interpolated at frame stamp)

    Returns:
      (x_mo, y_mo, yaw_mo): Map-to-odom SE(2) transformation
    """
    yaw_mo = normalize_angle(yaw_mb - yaw_ob)
    cos_mo = math.cos(yaw_mo)
    sin_mo = math.sin(yaw_mo)
    
    # In SE(2): x_mb = x_mo + x_ob * cos(yaw_mo) - y_ob * sin(yaw_mo)
    #          y_mb = y_mo + x_ob * sin(yaw_mo) + y_ob * cos(yaw_mo)
    x_mo = x_mb - (x_ob * cos_mo - y_ob * sin_mo)
    y_mo = y_mb - (x_ob * sin_mo + y_ob * cos_mo)
    return float(x_mo), float(y_mo), float(yaw_mo)

def compute_fused_pose_se2(x_mo: float, y_mo: float, yaw_mo: float,
                           x_ob_curr: float, y_ob_curr: float, yaw_ob_curr: float):
    """
    Apply SE(2) map->odom transformation to current odometry base pose:
      T_map_base_fused = T_map_odom @ T_odom_base_curr

    Returns:
      (x_fused, y_fused, yaw_fused): Robot base pose in map frame
    """
    yaw_fused = normalize_angle(yaw_mo + yaw_ob_curr)
    cos_mo = math.cos(yaw_mo)
    sin_mo = math.sin(yaw_mo)
    
    x_fused = x_mo + (x_ob_curr * cos_mo - y_ob_curr * sin_mo)
    y_fused = y_mo + (x_ob_curr * sin_mo + y_ob_curr * cos_mo)
    return float(x_fused), float(y_fused), float(yaw_fused)

def smooth_map_to_odom_se2(curr_x_mo: float, curr_y_mo: float, curr_yaw_mo: float,
                           target_x_mo: float, target_y_mo: float, target_yaw_mo: float,
                           alpha: float):
    """
    Exponentially smooth SE(2) map->odom transformation with circular yaw handling.
    """
    alpha = max(0.0, min(1.0, float(alpha)))
    new_x = curr_x_mo + alpha * (target_x_mo - curr_x_mo)
    new_y = curr_y_mo + alpha * (target_y_mo - curr_y_mo)
    
    yaw_err = normalize_angle(target_yaw_mo - curr_yaw_mo)
    new_yaw = normalize_angle(curr_yaw_mo + alpha * yaw_err)
    return float(new_x), float(new_y), float(new_yaw)

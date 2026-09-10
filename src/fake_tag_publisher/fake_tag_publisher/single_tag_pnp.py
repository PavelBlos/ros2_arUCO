"""
single_tag_pnp.py - High-Precision Single ArUco Tag Pose Estimation and Quality Metrics.

Features:
  - Strict camera calibration validation (zero silent fallbacks).
  - Dual-solution IPPE_SQUARE evaluation with planar ambiguity check.
  - Robust Levenberg-Marquardt refinement (solvePnPRefineLM) and fallback.
  - Reprojection error, viewing angle, distance, area, perimeter computation.
  - Explicit pose_valid boolean and rejection_reason codes.
  - Frame conversion from OpenCV Optical to ROS REP-103 camera_link.
"""

import math
import numpy as np
import cv2
from typing import Tuple, Dict, Any, Optional

try:
    from .geometry_transforms import optical_to_ros_matrix
except ImportError:
    from geometry_transforms import optical_to_ros_matrix

def validate_camera_calibration(camera_matrix: Optional[np.ndarray],
                                dist_coeffs: Optional[np.ndarray],
                                image_width: Optional[int] = None,
                                image_height: Optional[int] = None) -> Tuple[bool, str]:
    """
    Strictly validate camera intrinsic matrix and distortion coefficients.
    Returns (is_valid, reason).
    """
    if camera_matrix is None:
        return False, "camera_matrix_is_none"
    if not isinstance(camera_matrix, np.ndarray) or camera_matrix.shape != (3, 3):
        return False, f"invalid_camera_matrix_shape_{getattr(camera_matrix, 'shape', None)}"
    
    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])
    
    if not (math.isfinite(fx) and math.isfinite(fy) and math.isfinite(cx) and math.isfinite(cy)):
        return False, "non_finite_intrinsics"
    if fx <= 10.0 or fy <= 10.0:
        return False, f"unphysical_focal_length_fx_{fx}_fy_{fy}"
    
    if image_width is not None and image_height is not None:
        if cx <= 0 or cx >= image_width or cy <= 0 or cy >= image_height:
            return False, f"principal_point_out_of_bounds_cx_{cx}_cy_{cy}"
            
    if dist_coeffs is None or not isinstance(dist_coeffs, np.ndarray) or dist_coeffs.size < 4:
        return False, "invalid_distortion_coefficients"
        
    return True, "valid"

def get_marker_object_points(marker_size_m: float) -> np.ndarray:
    """
    4 corner coordinates of a square tag in its local frame:
    Center at (0, 0, 0), normal along +Z, corners in clockwise order (OpenCV standard).
    """
    half = float(marker_size_m) / 2.0
    return np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0]
    ], dtype=np.float64)

def solve_single_tag_ippe(corner_pts_2d: np.ndarray,
                          marker_size_m: float,
                          camera_matrix: np.ndarray,
                          dist_coeffs: np.ndarray,
                          prior_rvec: Optional[np.ndarray] = None,
                          prior_tvec: Optional[np.ndarray] = None) -> Dict[str, Any]:
    """
    Estimate 3D pose of a single square marker using SOLVEPNP_IPPE_SQUARE,
    evaluating dual planar solutions and refining with Levenberg-Marquardt.
    """
    calib_ok, calib_reason = validate_camera_calibration(camera_matrix, dist_coeffs)
    if not calib_ok:
        return {
            "pose_valid": False,
            "rejection_reason": f"invalid_calibration_{calib_reason}",
            "reproj_err": 999.0,
            "distance_m": 0.0,
            "viewing_angle_deg": 90.0,
            "ambiguity_status": 2,
            "ambiguity_ratio": 1.0,
            "marker_area_px": 0.0,
            "marker_perimeter_px": 0.0,
            "rvec": np.zeros(3, dtype=np.float64),
            "tvec": np.zeros(3, dtype=np.float64),
            "T_camOpt_tag": np.eye(4, dtype=np.float64),
            "T_cameraRos_tag": np.eye(4, dtype=np.float64)
        }

    corner_pts = np.asarray(corner_pts_2d, dtype=np.float64).reshape(-1, 2)
    if corner_pts.shape != (4, 2):
        return {
            "pose_valid": False,
            "rejection_reason": "invalid_corner_points_shape",
            "reproj_err": 999.0,
            "distance_m": 0.0,
            "viewing_angle_deg": 90.0,
            "ambiguity_status": 2,
            "ambiguity_ratio": 1.0,
            "marker_area_px": 0.0,
            "marker_perimeter_px": 0.0,
            "rvec": np.zeros(3, dtype=np.float64),
            "tvec": np.zeros(3, dtype=np.float64),
            "T_camOpt_tag": np.eye(4, dtype=np.float64),
            "T_cameraRos_tag": np.eye(4, dtype=np.float64)
        }

    area_px = float(cv2.contourArea(corner_pts.astype(np.float32)))
    perim_px = float(cv2.arcLength(corner_pts.astype(np.float32), True))
    
    if area_px < 80.0 or perim_px < 35.0:
        return {
            "pose_valid": False,
            "rejection_reason": f"marker_too_small_area_{area_px:.1f}",
            "reproj_err": 999.0,
            "distance_m": 0.0,
            "viewing_angle_deg": 90.0,
            "ambiguity_status": 2,
            "ambiguity_ratio": 1.0,
            "marker_area_px": area_px,
            "marker_perimeter_px": perim_px,
            "rvec": np.zeros(3, dtype=np.float64),
            "tvec": np.zeros(3, dtype=np.float64),
            "T_camOpt_tag": np.eye(4, dtype=np.float64),
            "T_cameraRos_tag": np.eye(4, dtype=np.float64)
        }

    obj_pts = get_marker_object_points(marker_size_m)

    # 1. Primary IPPE_SQUARE solve
    candidate_list = []
    try:
        ret_count, rvecs, tvecs, reproj_errs = cv2.solvePnPGeneric(
            obj_pts, corner_pts, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
    except Exception:
        rvecs, tvecs, ret_count = [], [], 0

    if ret_count > 0:
        for k in range(len(rvecs)):
            rv = rvecs[k].ravel()
            tv = tvecs[k].ravel()
            if tv[2] <= 0.05 or np.linalg.norm(rv) > 10.0:
                continue
            proj, _ = cv2.projectPoints(obj_pts, rv, tv, camera_matrix, dist_coeffs)
            err = float(np.sqrt(np.mean(np.sum((corner_pts - proj.reshape(-1, 2)) ** 2, axis=1))))
            candidate_list.append({"rvec": rv, "tvec": tv, "reproj_err": err})

    # 2. Also evaluate SOLVEPNP_ITERATIVE to guard against planar homography ambiguity
    try:
        ret_it, rv_it, tv_it = cv2.solvePnP(
            obj_pts, corner_pts, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        if ret_it and tv_it[2] > 0.05 and np.linalg.norm(rv_it) < 10.0:
            proj_it, _ = cv2.projectPoints(obj_pts, rv_it, tv_it, camera_matrix, dist_coeffs)
            err_it = float(np.sqrt(np.mean(np.sum((corner_pts - proj_it.reshape(-1, 2)) ** 2, axis=1))))
            candidate_list.append({"rvec": rv_it.ravel(), "tvec": tv_it.ravel(), "reproj_err": err_it})
    except Exception:
        pass

    if not candidate_list:
        return {
            "pose_valid": False,
            "rejection_reason": "no_valid_pnp_solution",
            "reproj_err": 999.0,
            "distance_m": 0.0,
            "viewing_angle_deg": 90.0,
            "ambiguity_status": 2,
            "ambiguity_ratio": 1.0,
            "marker_area_px": area_px,
            "marker_perimeter_px": perim_px,
            "rvec": np.zeros(3, dtype=np.float64),
            "tvec": np.zeros(3, dtype=np.float64),
            "T_camOpt_tag": np.eye(4, dtype=np.float64),
            "T_cameraRos_tag": np.eye(4, dtype=np.float64)
        }

    candidate_list.sort(key=lambda c: c["reproj_err"])
    ambiguity_ratio = (candidate_list[1]["reproj_err"] / max(1e-6, candidate_list[0]["reproj_err"])) if len(candidate_list) > 1 else 10.0
    
    chosen = candidate_list[0]
    ambiguity_status = 0
    if len(candidate_list) > 1 and ambiguity_ratio < 1.35:
        ambiguity_status = 1
        if prior_rvec is not None:
            diff0 = np.linalg.norm(candidate_list[0]["rvec"] - prior_rvec.ravel())
            diff1 = np.linalg.norm(candidate_list[1]["rvec"] - prior_rvec.ravel())
            if diff1 < diff0 * 0.7:
                chosen = candidate_list[1]

    # Non-linear refinement using solvePnPRefineLM
    refined_rvec = chosen["rvec"].copy().reshape(3, 1)
    refined_tvec = chosen["tvec"].copy().reshape(3, 1)
    try:
        refined_rvec, refined_tvec = cv2.solvePnPRefineLM(
            obj_pts, corner_pts, camera_matrix, dist_coeffs,
            refined_rvec, refined_tvec
        )
    except Exception:
        pass

    final_rvec = refined_rvec.ravel()
    final_tvec = refined_tvec.ravel()

    # Recompute final reprojection error
    final_proj, _ = cv2.projectPoints(obj_pts, final_rvec, final_tvec, camera_matrix, dist_coeffs)
    final_reproj_err = float(np.sqrt(np.mean(np.sum((corner_pts - final_proj.reshape(-1, 2)) ** 2, axis=1))))
    final_dist = float(np.linalg.norm(final_tvec))

    # Compute viewing angle
    R_final, _ = cv2.Rodrigues(final_rvec)
    tag_normal = R_final @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    ray = final_tvec / max(1e-6, final_dist)
    cos_v = float(np.dot(-ray, tag_normal))
    viewing_angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, cos_v))))

    # Validity criteria
    pose_valid = True
    rejection_reason = ""
    
    if final_reproj_err > 4.0:
        pose_valid = False
        rejection_reason = f"high_reprojection_error_{final_reproj_err:.2f}px"
    elif viewing_angle_deg > 65.0:
        pose_valid = False
        rejection_reason = f"high_viewing_angle_{viewing_angle_deg:.1f}deg"
    elif final_dist < 0.15 or final_dist > 6.0:
        pose_valid = False
        rejection_reason = f"unphysical_distance_{final_dist:.2f}m"

    # Construct SE(3) in Optical frame
    T_camOpt_tag = np.eye(4, dtype=np.float64)
    T_camOpt_tag[:3, :3] = R_final
    T_camOpt_tag[:3, 3] = final_tvec

    # Construct SE(3) in ROS REP-103 camera_link frame
    T_cameraRos_tag = optical_to_ros_matrix() @ T_camOpt_tag

    return {
        "pose_valid": pose_valid,
        "rejection_reason": rejection_reason,
        "reproj_err": final_reproj_err,
        "distance_m": final_dist,
        "viewing_angle_deg": viewing_angle_deg,
        "ambiguity_status": ambiguity_status,
        "ambiguity_ratio": float(ambiguity_ratio),
        "marker_area_px": area_px,
        "marker_perimeter_px": perim_px,
        "rvec": final_rvec,
        "tvec": final_tvec,
        "T_camOpt_tag": T_camOpt_tag,
        "T_cameraRos_tag": T_cameraRos_tag
    }

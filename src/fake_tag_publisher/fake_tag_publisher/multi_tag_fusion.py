"""
multi_tag_fusion.py - Deterministic Multi-Tag Consensus and Joint 3D-2D PnP Optimization.

Features:
  - Dynamic observation covariance modeling (reprojection error, area, distance, viewing angle).
  - SE(2) Jacobian odometry process noise propagation.
  - Two-tag conflict gate via Mahalanobis distance with odometry prior disambiguation.
  - Tag-level IPPE hypothesis evaluation and whole-tag consensus.
  - Joint Levenberg-Marquardt optimization (solvePnPRefineLM) across all inlier corners.
  - Post-solve per-tag residual verification (discarding whole bad tags, never partial corners).
  - Exact SE(2) map->odom update composition.
"""

import math
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from typing import Dict, List, Any, Optional, Tuple

try:
    from .geometry_transforms import (
        normalize_angle,
        invert_transform,
        pose_to_matrix,
        matrix_to_pose,
        camera_pose_from_tag,
        base_pose_from_camera,
        compute_map_to_odom_se2,
        compute_fused_pose_se2,
        optical_to_ros_matrix,
        ros_to_optical_matrix
    )
    from .single_tag_pnp import get_marker_object_points, solve_single_tag_ippe
except ImportError:
    from geometry_transforms import (
        normalize_angle,
        invert_transform,
        pose_to_matrix,
        matrix_to_pose,
        camera_pose_from_tag,
        base_pose_from_camera,
        compute_map_to_odom_se2,
        compute_fused_pose_se2,
        optical_to_ros_matrix,
        ros_to_optical_matrix
    )
    from single_tag_pnp import get_marker_object_points, solve_single_tag_ippe

CHI2_3_95 = 7.815  # 95% confidence threshold for 3 degrees of freedom (x, y, yaw)
CHI2_3_999 = 16.266

def compute_visual_covariance(distance_m: float,
                              viewing_angle_deg: float,
                              reproj_err_px: float,
                              marker_area_px: float,
                              sigma_xy_0: float = 0.010,
                              sigma_yaw_0: float = 0.020,
                              d0: float = 2.0,
                              k_err: float = 0.50,
                              cos_min: float = 0.50) -> np.ndarray:
    """
    Compute 3x3 covariance matrix (x, y, yaw) for a single visual tag observation.
    Clamped against excessive sensitivity, regularized with positive floor.
    """
    cos_v = max(cos_min, math.cos(math.radians(min(65.0, abs(viewing_angle_deg)))))
    d_ratio = max(0.2, float(distance_m)) / d0
    err_factor = 1.0 + k_err * max(0.0, float(reproj_err_px))
    
    # Area scaling (normalized against 3000 px^2, clamped)
    area_factor = 1.0
    if marker_area_px > 50.0:
        area_factor = max(0.5, min(2.0, math.sqrt(3000.0 / marker_area_px)))

    sigma_xy = max(0.005, sigma_xy_0 * err_factor * d_ratio * area_factor / cos_v)
    sigma_yaw = max(0.005, sigma_yaw_0 * err_factor * d_ratio / cos_v)

    cov = np.diag([sigma_xy ** 2, sigma_xy ** 2, sigma_yaw ** 2])
    cov += np.eye(3) * 1e-6  # Regularization
    return cov

def propagate_odometry_covariance(prev_cov: np.ndarray,
                                  dx_step: float,
                                  dy_step: float,
                                  dyaw_step: float,
                                  prev_yaw: float,
                                  dt: float,
                                  q_lin: float = 0.02,
                                  q_ang: float = 0.03,
                                  drift_floor: float = 1e-4) -> np.ndarray:
    """
    Propagate odometry covariance via SE(2) Jacobian:
      p_{k} = p_{k-1} + R(yaw_{k-1}) @ [dx, dy]
      yaw_{k} = yaw_{k-1} + dyaw
    """
    cos_y = math.cos(prev_yaw)
    sin_y = math.sin(prev_yaw)
    
    # Jacobian wrt [x, y, yaw]
    J = np.eye(3, dtype=np.float64)
    J[0, 2] = -dx_step * sin_y - dy_step * cos_y
    J[1, 2] =  dx_step * cos_y - dy_step * sin_y

    dist_step = math.hypot(dx_step, dy_step)
    Q = np.diag([
        q_lin * dist_step + drift_floor * dt,
        q_lin * dist_step + drift_floor * dt,
        q_ang * abs(dyaw_step) + drift_floor * dt
    ])

    return J @ prev_cov @ J.T + Q

def compute_mahalanobis_distance(x1: float, y1: float, yaw1: float, cov1: np.ndarray,
                                 x2: float, y2: float, yaw2: float, cov2: np.ndarray) -> Tuple[float, np.ndarray]:
    """
    Compute Mahalanobis distance D_M between two planar poses with circular yaw difference:
      e = [x1 - x2, y1 - y2, normalize_angle(yaw1 - yaw2)]^T
      D_M^2 = e^T @ (cov1 + cov2)^-1 @ e
    """
    e = np.array([
        x1 - x2,
        y1 - y2,
        normalize_angle(yaw1 - yaw2)
    ], dtype=np.float64)

    total_cov = cov1 + cov2
    # Invert with pseudo-inverse for robust conditioning
    inv_cov = np.linalg.pinv(total_cov)
    d_sq = float(e.T @ inv_cov @ e)
    d_m = math.sqrt(max(0.0, d_sq))
    return d_m, e

class MultiTagFusion:
    def __init__(self, tau_inlier_px: float = 2.5, tau_tag_max_px: float = 3.5):
        self.tau_inlier_px = float(tau_inlier_px)
        self.tau_tag_max_px = float(tau_tag_max_px)

    def process_frame(self,
                      detections: List[Dict[str, Any]],
                      active_tags_db: Dict[str, Any],
                      camera_matrix: np.ndarray,
                      dist_coeffs: np.ndarray,
                      T_base_cam: np.ndarray,
                      odom_at_stamp: Tuple[float, float, float],
                      pred_odom_cov: Optional[np.ndarray] = None,
                      expected_epoch: Optional[int] = None,
                      expected_revision: Optional[int] = None,
                      expected_sha256: Optional[str] = None,
                      frame_epoch: Optional[int] = None,
                      frame_revision: Optional[int] = None,
                      frame_sha256: Optional[str] = None) -> Dict[str, Any]:
        """
        Main fusion entrypoint for a single video frame.

        Args:
          detections: List of detection dicts (tag_id, pose_valid, corners_px, etc.)
          active_tags_db: Dict of active confirmed tags from TagRegistry
          camera_matrix, dist_coeffs: Camera calibration
          T_base_cam: SE(3) camera extrinsics (base_link -> camera_link)
          odom_at_stamp: (x_ob, y_ob, yaw_ob) odometry pose at frame capture_stamp
          pred_odom_cov: (3x3) predicted odometry covariance matrix
          expected_epoch, expected_revision, expected_sha256: active tag map config metadata
          frame_epoch, frame_revision, frame_sha256: metadata from detection frame

        Returns:
          Result dictionary with fused base pose, inliers, outliers, residuals, and diagnostics.
        """
        # 0. Check hot-reload configuration handshake synchronization
        metadata_mismatch = (
            expected_epoch is not None and frame_epoch != expected_epoch
        ) or (
            expected_revision is not None and frame_revision != expected_revision
        ) or (
            bool(expected_sha256) and frame_sha256 != expected_sha256
        )
        if metadata_mismatch:
                return {
                    "status": "revision_mismatch",
                    "fused_base_pose": None,
                    "inlier_ids": [],
                    "rejected_ids": [str(d.get("tag_id")) for d in detections],
                    "rejection_reasons": {str(d.get("tag_id")): "revision_mismatch" for d in detections},
                    "reproj_rms_px": 0.0,
                    "multi_tag_used": False,
                    "covariance": pred_odom_cov if pred_odom_cov is not None else np.diag([0.05**2, 0.05**2, 0.05**2])
                }

        # 1. Filter valid confirmed detections
        valid_candidates = []
        for det in detections:
            tid = str(det.get("tag_id"))
            if not det.get("pose_valid", False):
                continue
            if tid not in active_tags_db:
                continue
            tag_info = active_tags_db[tid]
            if not tag_info.get("enabled", True) or tag_info.get("state") != "confirmed":
                continue
            
            valid_candidates.append({
                "detection": det,
                "tag_info": tag_info,
                "tag_id": tid
            })

        if not valid_candidates:
            return {
                "status": "no_confirmed_tags",
                "fused_base_pose": None,
                "inlier_ids": [],
                "rejected_ids": [str(d.get("tag_id")) for d in detections if not d.get("pose_valid", False)],
                "rejection_reasons": {str(d.get("tag_id")): d.get("rejection_reason", "invalid") for d in detections if not d.get("pose_valid", False)},
                "reproj_rms_px": 0.0,
                "multi_tag_used": False,
                "covariance": pred_odom_cov if pred_odom_cov is not None else np.diag([0.05**2, 0.05**2, 0.05**2])
            }

        x_ob, y_ob, yaw_ob = odom_at_stamp

        # 2. Extract independent candidate poses for each valid tag
        candidate_poses = []
        for cand in valid_candidates:
            det = cand["detection"]
            tag_info = cand["tag_info"]
            tid = cand["tag_id"]
            
            # T_map_tag
            p = tag_info["pose"]
            T_map_tag = pose_to_matrix(p["x"], p["y"], p["z"], p["roll"], p["pitch"], p["yaw"])
            
            # T_camOpt_tag from detection
            corners_2d = np.array(det["corners_px"], dtype=np.float64).reshape(4, 2)
            marker_size_m = float(det.get("marker_size_mm", 100.0)) / 1000.0
            obj_pts = get_marker_object_points(marker_size_m)
            
            # Reconstruct or use T_cameraRos_tag
            T_cameraRos_tag = det.get("T_cameraRos_tag")
            if T_cameraRos_tag is None and "pose_position" in det:
                pos = det.get("pose_position", [0, 0, 0])
                rot = det.get("pose_orientation", [0, 0, 0, 1])
                T_cameraRos_tag = np.eye(4, dtype=np.float64)
                T_cameraRos_tag[:3, :3] = cv2.Rodrigues(cv2.Rodrigues(np.array(rot))[0])[0] if len(rot) == 3 else R.from_quat(rot).as_matrix()
                T_cameraRos_tag[:3, 3] = pos
            elif T_cameraRos_tag is None and hasattr(det, "pose"):
                pos = [det.pose.position.x, det.pose.position.y, det.pose.position.z]
                rot = [det.pose.orientation.x, det.pose.orientation.y, det.pose.orientation.z, det.pose.orientation.w]
                T_cameraRos_tag = np.eye(4, dtype=np.float64)
                T_cameraRos_tag[:3, :3] = R.from_quat(rot).as_matrix()
                T_cameraRos_tag[:3, 3] = pos
            elif T_cameraRos_tag is None:
                res_ippe = solve_single_tag_ippe(corners_2d, marker_size_m, camera_matrix, dist_coeffs)
                T_cameraRos_tag = res_ippe["T_cameraRos_tag"]
            
            # T_map_camRos = T_map_tag @ invert_transform(T_cameraRos_tag)
            T_map_camRos = camera_pose_from_tag(T_map_tag, T_cameraRos_tag)
            
            # T_map_camOpt = T_map_camRos @ optical_to_ros_matrix()
            T_map_camOpt = T_map_camRos @ optical_to_ros_matrix()
            
            # T_map_base = T_map_camRos @ invert_transform(T_base_cam)
            T_map_base = base_pose_from_camera(T_map_camRos, T_base_cam)
            
            x_b, y_b, z_b, r_b, p_b, yaw_b = matrix_to_pose(T_map_base)
            
            cov_vis = compute_visual_covariance(
                distance_m=det.get("distance_m", 2.0),
                viewing_angle_deg=det.get("viewing_angle_deg", 0.0),
                reproj_err_px=det.get("reproj_err", 1.0),
                marker_area_px=det.get("marker_area_px", 1000.0)
            )
            
            candidate_poses.append({
                "tag_id": tid,
                "cand": cand,
                "T_map_tag": T_map_tag,
                "T_cameraRos_tag": T_cameraRos_tag,
                "T_map_camRos": T_map_camRos,
                "T_map_camOpt": T_map_camOpt,
                "T_map_base": T_map_base,
                "base_pose_se2": (x_b, y_b, yaw_b),
                "cov_vis": cov_vis,
                "corners_2d": corners_2d,
                "obj_pts": obj_pts
            })

        # 3. Handle single tag observation
        if len(candidate_poses) == 1:
            c = candidate_poses[0]
            x_b, y_b, yaw_b = c["base_pose_se2"]
            return {
                "status": "single_tag_ok",
                "fused_base_pose": (x_b, y_b, yaw_b),
                "inlier_ids": [c["tag_id"]],
                "rejected_ids": [],
                "rejection_reasons": {},
                "reproj_rms_px": float(c["cand"]["detection"].get("reproj_err", 0.0)),
                "multi_tag_used": False,
                "covariance": c["cov_vis"]
            }

        # 4. Handle 2-tag observation (Mahalanobis conflict gate)
        if len(candidate_poses) == 2:
            c1, c2 = candidate_poses[0], candidate_poses[1]
            x1, y1, yaw1 = c1["base_pose_se2"]
            x2, y2, yaw2 = c2["base_pose_se2"]
            
            d_m12, _ = compute_mahalanobis_distance(x1, y1, yaw1, c1["cov_vis"], x2, y2, yaw2, c2["cov_vis"])
            
            if d_m12 <= math.sqrt(CHI2_3_95):
                # Consistent! Proceed to joint refinement across both
                return self._solve_joint_pnp(candidate_poses, camera_matrix, dist_coeffs, T_base_cam)
            else:
                # Conflict between the two tags!
                if pred_odom_cov is not None:
                    # Compare with odometry prediction
                    x_pred, y_pred, yaw_pred = compute_fused_pose_se2(
                        0.0, 0.0, 0.0, x_ob, y_ob, yaw_ob  # relative check
                    )
                    # Use absolute predicted pose if available
                    d1_pred, _ = compute_mahalanobis_distance(x1, y1, yaw1, c1["cov_vis"], x_ob, y_ob, yaw_ob, pred_odom_cov)
                    d2_pred, _ = compute_mahalanobis_distance(x2, y2, yaw2, c2["cov_vis"], x_ob, y_ob, yaw_ob, pred_odom_cov)
                    
                    if d1_pred <= math.sqrt(CHI2_3_95) and d2_pred > math.sqrt(CHI2_3_999):
                        return {
                            "status": "conflict_resolved_tag1",
                            "fused_base_pose": (x1, y1, yaw1),
                            "inlier_ids": [c1["tag_id"]],
                            "rejected_ids": [c2["tag_id"]],
                            "rejection_reasons": {c2["tag_id"]: f"conflict_mahalanobis_{d2_pred:.2f}"},
                            "reproj_rms_px": float(c1["cand"]["detection"].get("reproj_err", 0.0)),
                            "multi_tag_used": False,
                            "covariance": c1["cov_vis"]
                        }
                    elif d2_pred <= math.sqrt(CHI2_3_95) and d1_pred > math.sqrt(CHI2_3_999):
                        return {
                            "status": "conflict_resolved_tag2",
                            "fused_base_pose": (x2, y2, yaw2),
                            "inlier_ids": [c2["tag_id"]],
                            "rejected_ids": [c1["tag_id"]],
                            "rejection_reasons": {c1["tag_id"]: f"conflict_mahalanobis_{d1_pred:.2f}"},
                            "reproj_rms_px": float(c2["cand"]["detection"].get("reproj_err", 0.0)),
                            "multi_tag_used": False,
                            "covariance": c2["cov_vis"]
                        }

                # Unresolved conflict: hold odometry, flag multi_tag_conflict!
                return {
                    "status": "multi_tag_conflict",
                    "fused_base_pose": None,
                    "inlier_ids": [],
                    "rejected_ids": [c1["tag_id"], c2["tag_id"]],
                    "rejection_reasons": {
                        c1["tag_id"]: f"multi_tag_conflict_d_m_{d_m12:.2f}",
                        c2["tag_id"]: f"multi_tag_conflict_d_m_{d_m12:.2f}"
                    },
                    "reproj_rms_px": 999.0,
                    "multi_tag_used": False,
                    "covariance": pred_odom_cov if pred_odom_cov is not None else np.diag([0.1**2, 0.1**2, 0.1**2])
                }

        # 5. Handle N >= 3: Tag-level hypothesis consensus
        best_hypothesis = None
        best_inliers = []
        best_inlier_err_sum = 9999.0

        for cand_idx, hyp in enumerate(candidate_poses):
            # Test hypothesis: camera optical pose in map frame
            T_map_camOpt_hyp = hyp["T_map_camOpt"]
            T_camOpt_map_hyp = invert_transform(T_map_camOpt_hyp)
            rvec_hyp, _ = cv2.Rodrigues(T_camOpt_map_hyp[:3, :3])
            tvec_hyp = T_camOpt_map_hyp[:3, 3]

            # Project corners of all candidates and evaluate whole-tag inliers
            inliers = []
            err_sum = 0.0
            
            for other_cand in candidate_poses:
                # 3D corners of other_cand in map frame:
                # P_map = T_map_tag @ P_local
                T_mt = other_cand["T_map_tag"]
                obj_pts = other_cand["obj_pts"]
                pts_4d = np.hstack([obj_pts, np.ones((4, 1))])
                corners_3d_map = (T_mt @ pts_4d.T).T[:, :3]

                # Project using hyp camera
                proj, _ = cv2.projectPoints(corners_3d_map, rvec_hyp, tvec_hyp, camera_matrix, dist_coeffs)
                diff = other_cand["corners_2d"] - proj.reshape(4, 2)
                tag_rms = float(np.sqrt(np.mean(np.sum(diff ** 2, axis=1))))

                if tag_rms <= self.tau_inlier_px:
                    inliers.append((other_cand, tag_rms))
                    err_sum += tag_rms

            if len(inliers) > len(best_inliers) or (len(inliers) == len(best_inliers) and err_sum < best_inlier_err_sum):
                best_inliers = inliers
                best_inlier_err_sum = err_sum
                best_hypothesis = hyp

        inlier_candidates = [item[0] for item in best_inliers]
        if len(inlier_candidates) < 2:
            # Consensus failed across N >= 3 tags!
            # Check if odometry can unambiguously select one candidate
            chosen = None
            if pred_odom_cov is not None:
                d_m_list = []
                for c in candidate_poses:
                    xc, yc, yawc = c["base_pose_se2"]
                    dm, _ = compute_mahalanobis_distance(xc, yc, yawc, c["cov_vis"], x_ob, y_ob, yaw_ob, pred_odom_cov)
                    d_m_list.append((c, dm))

                d_m_list.sort(key=lambda x: x[1])
                best_c, best_dm = d_m_list[0]
                second_dm = d_m_list[1][1] if len(d_m_list) > 1 else 999.0

                # Must be consistent with odom and clearly separated from runner-up
                if best_dm <= math.sqrt(CHI2_3_95) and second_dm > math.sqrt(CHI2_3_999):
                    chosen = best_c

            if chosen is not None:
                x_b, y_b, yaw_b = chosen["base_pose_se2"]
                return {
                    "status": "consensus_resolved_by_odometry",
                    "fused_base_pose": (x_b, y_b, yaw_b),
                    "inlier_ids": [chosen["tag_id"]],
                    "rejected_ids": [c["tag_id"] for c in candidate_poses if c["tag_id"] != chosen["tag_id"]],
                    "rejection_reasons": {c["tag_id"]: "consensus_outlier" for c in candidate_poses if c["tag_id"] != chosen["tag_id"]},
                    "reproj_rms_px": float(chosen["cand"]["detection"].get("reproj_err", 0.0)),
                    "multi_tag_used": False,
                    "covariance": chosen["cov_vis"]
                }
            else:
                # Ambiguous conflict: hold dead reckoning odometry, do NOT jump
                return {
                    "status": "multi_tag_conflict",
                    "fused_base_pose": None,
                    "inlier_ids": [],
                    "rejected_ids": [c["tag_id"] for c in candidate_poses],
                    "rejection_reasons": {c["tag_id"]: "multi_tag_consensus_failure" for c in candidate_poses},
                    "reproj_rms_px": 999.0,
                    "multi_tag_used": False,
                    "covariance": pred_odom_cov if pred_odom_cov is not None else np.diag([0.1**2, 0.1**2, 0.1**2])
                }

        # 6. Joint solvePnPRefineLM across all inlier corners
        return self._solve_joint_pnp(inlier_candidates, camera_matrix, dist_coeffs, T_base_cam)

    def _solve_joint_pnp(self,
                         inlier_candidates: List[Dict[str, Any]],
                         camera_matrix: np.ndarray,
                         dist_coeffs: np.ndarray,
                         T_base_cam: np.ndarray) -> Dict[str, Any]:
        """
        Jointly optimize camera pose using all 3D-2D corner correspondences from whole tag inliers.
        Verifies individual tag residuals post-solve, dropping worst whole tag if exceeding threshold.
        """
        active_inliers = list(inlier_candidates)
        rejected_ids = []
        rejection_reasons = {}

        for iteration in range(3):  # Max 3 prune iterations
            if len(active_inliers) < 2:
                break

            all_3d = []
            all_2d = []
            tag_indices = {}

            idx = 0
            for c in active_inliers:
                T_mt = c["T_map_tag"]
                obj_pts = c["obj_pts"]
                pts_4d = np.hstack([obj_pts, np.ones((4, 1))])
                corners_3d = (T_mt @ pts_4d.T).T[:, :3]
                
                all_3d.append(corners_3d)
                all_2d.append(c["corners_2d"])
                tag_indices[c["tag_id"]] = (idx, idx + 4)
                idx += 4

            all_3d_arr = np.vstack(all_3d).astype(np.float64)
            all_2d_arr = np.vstack(all_2d).astype(np.float64)

            # Prior from first inlier (in OpenCV camera optical frame)
            T_map_camOpt_prior = active_inliers[0]["T_map_camOpt"]
            T_camOpt_map_prior = invert_transform(T_map_camOpt_prior)
            rvec_init, _ = cv2.Rodrigues(T_camOpt_map_prior[:3, :3])
            tvec_init = T_camOpt_map_prior[:3, 3].reshape(3, 1)

            # Run Levenberg-Marquardt
            try:
                rvec_ref, tvec_ref = cv2.solvePnPRefineLM(
                    all_3d_arr, all_2d_arr, camera_matrix, dist_coeffs,
                    rvec_init.copy(), tvec_init.copy()
                )
            except Exception:
                rvec_ref, tvec_ref = rvec_init, tvec_init

            # Compute post-solve individual tag residuals
            proj_all, _ = cv2.projectPoints(all_3d_arr, rvec_ref, tvec_ref, camera_matrix, dist_coeffs)
            diffs = all_2d_arr - proj_all.reshape(-1, 2)
            
            worst_tag_id = None
            worst_tag_rms = 0.0

            for tid, (start_i, end_i) in tag_indices.items():
                tag_diff = diffs[start_i:end_i]
                tag_rms = float(np.sqrt(np.mean(np.sum(tag_diff ** 2, axis=1))))
                if tag_rms > worst_tag_rms:
                    worst_tag_rms = tag_rms
                    worst_tag_id = tid

            if worst_tag_rms > self.tau_tag_max_px and len(active_inliers) > 2:
                # Discard worst whole tag and re-solve
                rejected_ids.append(worst_tag_id)
                rejection_reasons[worst_tag_id] = f"post_joint_residual_{worst_tag_rms:.2f}px"
                active_inliers = [c for c in active_inliers if c["tag_id"] != worst_tag_id]
                continue
            else:
                # Accept solution!
                R_cam_map, _ = cv2.Rodrigues(rvec_ref)
                T_camOpt_map_ref = np.eye(4, dtype=np.float64)
                T_camOpt_map_ref[:3, :3] = R_cam_map
                T_camOpt_map_ref[:3, 3] = tvec_ref.ravel()

                T_map_camOpt_final = invert_transform(T_camOpt_map_ref)
                T_map_camRos_final = T_map_camOpt_final @ ros_to_optical_matrix()
                T_map_base_final = base_pose_from_camera(T_map_camRos_final, T_base_cam)
                x_b, y_b, z_b, r_b, p_b, yaw_b = matrix_to_pose(T_map_base_final)

                overall_rms = float(np.sqrt(np.mean(np.sum(diffs ** 2, axis=1))))

                # Combined covariance (scaled down by sqrt(N_inliers))
                N = len(active_inliers)
                avg_cov = np.mean([c["cov_vis"] for c in active_inliers], axis=0) / math.sqrt(N)

                return {
                    "status": "multi_tag_ok",
                    "fused_base_pose": (x_b, y_b, yaw_b),
                    "inlier_ids": [c["tag_id"] for c in active_inliers],
                    "rejected_ids": rejected_ids,
                    "rejection_reasons": rejection_reasons,
                    "reproj_rms_px": overall_rms,
                    "multi_tag_used": True,
                    "covariance": avg_cov
                }

        # Fallback if pruning left only 1 tag
        c = active_inliers[0]
        x_b, y_b, yaw_b = c["base_pose_se2"]
        return {
            "status": "single_tag_fallback",
            "fused_base_pose": (x_b, y_b, yaw_b),
            "inlier_ids": [c["tag_id"]],
            "rejected_ids": rejected_ids,
            "rejection_reasons": rejection_reasons,
            "reproj_rms_px": float(c["cand"]["detection"].get("reproj_err", 0.0)),
            "multi_tag_used": False,
            "covariance": c["cov_vis"]
        }

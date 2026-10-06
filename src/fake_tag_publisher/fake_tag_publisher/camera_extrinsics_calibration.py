"""Robust hand-eye estimate for a stationary robot under a mapped marker."""

import math
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rotation

try:
    from .geometry_transforms import (
        invert_transform, optical_to_ros_matrix, ros_to_optical_matrix,
    )
    from .single_tag_pnp import get_marker_object_points
except ImportError:
    from geometry_transforms import (
        invert_transform, optical_to_ros_matrix, ros_to_optical_matrix,
    )
    from single_tag_pnp import get_marker_object_points


def estimate_base_camera(sample_camera_tag, T_map_base, T_map_tag, min_samples=8):
    """Estimate T_base_cam from known base/tag map poses and camera-tag samples."""
    matrices = [np.asarray(item, dtype=float) for item in sample_camera_tag]
    matrices = [item for item in matrices if item.shape == (4, 4) and np.all(np.isfinite(item))]
    if len(matrices) < int(min_samples):
        raise ValueError(f"Need at least {int(min_samples)} valid frames")

    base_from_map = invert_transform(np.asarray(T_map_base, dtype=float))
    candidates = np.asarray([
        base_from_map @ np.asarray(T_map_tag, dtype=float) @ invert_transform(item)
        for item in matrices
    ])
    translations = candidates[:, :3, 3]
    rotations = Rotation.from_matrix(candidates[:, :3, :3])

    median_translation = np.median(translations, axis=0)
    preliminary_rotation = rotations.mean()
    translation_error = np.linalg.norm(translations - median_translation, axis=1)
    rotation_error = (preliminary_rotation.inv() * rotations).magnitude()

    trans_median = float(np.median(translation_error))
    trans_mad = float(np.median(np.abs(translation_error - trans_median)))
    rot_median = float(np.median(rotation_error))
    rot_mad = float(np.median(np.abs(rotation_error - rot_median)))
    translation_gate = max(0.008, trans_median + 3.5 * max(trans_mad, 0.001))
    rotation_gate = max(math.radians(2.0), rot_median + 3.5 * max(rot_mad, math.radians(0.25)))
    inliers = (translation_error <= translation_gate) & (rotation_error <= rotation_gate)
    if int(np.count_nonzero(inliers)) < int(min_samples):
        raise ValueError("Camera-pose samples are inconsistent; keep the robot still and retry")

    accepted = candidates[inliers]
    result = np.eye(4, dtype=float)
    result[:3, 3] = np.median(accepted[:, :3, 3], axis=0)
    result[:3, :3] = Rotation.from_matrix(accepted[:, :3, :3]).mean().as_matrix()

    accepted_translation_error = np.linalg.norm(
        accepted[:, :3, 3] - result[:3, 3], axis=1
    )
    accepted_rotation_error = (
        Rotation.from_matrix(result[:3, :3]).inv()
        * Rotation.from_matrix(accepted[:, :3, :3])
    ).magnitude()
    diagnostics = {
        "samples": len(matrices),
        "inliers": int(len(accepted)),
        "translation_spread_mm": round(float(np.sqrt(np.mean(accepted_translation_error ** 2))) * 1000.0, 2),
        "rotation_spread_deg": round(math.degrees(float(np.sqrt(np.mean(accepted_rotation_error ** 2)))), 2),
    }
    return result, diagnostics


def _mapped_tag_corners(tag):
    pose = tag['pose']
    size_m = float(tag.get('size_mm', tag.get('marker_size_m', 0.1) * 1000.0)) / 1000.0
    local = get_marker_object_points(size_m)
    T_map_tag = np.eye(4, dtype=float)
    T_map_tag[:3, :3] = Rotation.from_euler(
        'xyz', [float(pose.get('roll', math.pi)),
                float(pose.get('pitch', 0.0)), float(pose.get('yaw', 0.0))]
    ).as_matrix()
    T_map_tag[:3, 3] = [float(pose['x']), float(pose['y']), float(pose['z'])]
    return (T_map_tag @ np.column_stack([local, np.ones(4)]).T).T[:, :3]


def _robust_transform_average(candidates, min_samples):
    candidates = np.asarray(candidates, dtype=float)
    translations = candidates[:, :3, 3]
    rotations = Rotation.from_matrix(candidates[:, :3, :3])
    median_translation = np.median(translations, axis=0)
    preliminary_rotation = rotations.mean()
    translation_error = np.linalg.norm(translations - median_translation, axis=1)
    rotation_error = (preliminary_rotation.inv() * rotations).magnitude()
    trans_median = float(np.median(translation_error))
    trans_mad = float(np.median(np.abs(translation_error - trans_median)))
    rot_median = float(np.median(rotation_error))
    rot_mad = float(np.median(np.abs(rotation_error - rot_median)))
    inliers = (
        (translation_error <= max(0.008, trans_median + 3.5 * max(trans_mad, 0.001)))
        & (rotation_error <= max(
            math.radians(2.0), rot_median + 3.5 * max(rot_mad, math.radians(0.25))
        ))
    )
    if int(np.count_nonzero(inliers)) < int(min_samples):
        raise ValueError('Camera-pose samples are inconsistent; keep the robot still and retry')
    accepted = candidates[inliers]
    result = np.eye(4, dtype=float)
    result[:3, 3] = np.median(accepted[:, :3, 3], axis=0)
    result[:3, :3] = Rotation.from_matrix(accepted[:, :3, :3]).mean().as_matrix()
    t_error = np.linalg.norm(accepted[:, :3, 3] - result[:3, 3], axis=1)
    r_error = (
        Rotation.from_matrix(result[:3, :3]).inv()
        * Rotation.from_matrix(accepted[:, :3, :3])
    ).magnitude()
    return result, inliers, t_error, r_error


def validate_camera_mount(T_base_cam, max_horizontal_m=0.35,
                          min_height_m=-0.05, max_height_m=1.0):
    """Reject wrong-marker, wrong-unit and planar-PnP mirror solutions."""
    matrix = np.asarray(T_base_cam, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError('Camera transform is not finite')
    x, y, z = matrix[:3, 3]
    horizontal = float(math.hypot(x, y))
    if horizontal > float(max_horizontal_m):
        raise ValueError(
            f'Calculated horizontal camera offset is {horizontal:.2f} m. '
            'The robot is not centered under the selected tag, the tag ID is wrong, '
            'or robot coordinates were entered in millimetres instead of metres.'
        )
    if not float(min_height_m) <= z <= float(max_height_m):
        raise ValueError(
            f'Calculated camera height is {z:.2f} m; expected '
            f'{min_height_m:.2f}…{max_height_m:.2f} m above base_link.'
        )
    optical_axis = matrix[:3, :3] @ optical_to_ros_matrix()[:3, :3] @ np.array([0., 0., 1.])
    if float(optical_axis[2]) < 0.55:
        raise ValueError('Calculated camera optical axis does not point upward')
    return {'horizontal_offset_m': horizontal, 'camera_height_m': float(z)}


def estimate_base_camera_from_frames(frames, tags, T_map_base, camera_matrix,
                                     dist_coeffs, prior_base_camera=None,
                                     min_frames=8, min_tags=2):
    """Estimate the mount using joint mapped corners instead of single-tag rotation."""
    T_map_base = np.asarray(T_map_base, dtype=float)
    K = np.asarray(camera_matrix, dtype=float)
    distortion = np.asarray(dist_coeffs, dtype=float)
    prior = None if prior_base_camera is None else np.asarray(prior_base_camera, dtype=float)
    candidates, reprojection = [], []
    used_tag_ids = set()

    for frame in frames:
        object_points, image_points, frame_ids = [], [], set()
        for detection in frame.get('detections', []):
            tag_id = str(detection.get('tag_id'))
            tag = tags.get(tag_id)
            corners = np.asarray(detection.get('corners_px', []), dtype=float)
            if (not tag or tag.get('state') != 'confirmed' or not tag.get('enabled', True)
                    or not detection.get('pose_valid', False) or corners.size != 8):
                continue
            object_points.append(_mapped_tag_corners(tag))
            image_points.append(corners.reshape(4, 2))
            frame_ids.add(tag_id)
        if len(frame_ids) < int(min_tags):
            continue
        obj = np.vstack(object_points).astype(np.float64)
        img = np.vstack(image_points).astype(np.float64)
        try:
            if prior is not None:
                T_map_cam_ros = T_map_base @ prior
                T_map_cam_opt = T_map_cam_ros @ optical_to_ros_matrix()
                T_cam_opt_map = invert_transform(T_map_cam_opt)
                rvec, _ = cv2.Rodrigues(T_cam_opt_map[:3, :3])
                tvec = T_cam_opt_map[:3, 3].reshape(3, 1)
                ok, rvec, tvec = cv2.solvePnP(
                    obj, img, K, distortion, rvec, tvec, True, cv2.SOLVEPNP_ITERATIVE
                )
            else:
                ok, rvec, tvec = cv2.solvePnP(
                    obj, img, K, distortion, flags=cv2.SOLVEPNP_ITERATIVE
                )
            if not ok:
                continue
            rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, distortion, rvec, tvec)
            projected, _ = cv2.projectPoints(obj, rvec, tvec, K, distortion)
            rms = float(np.sqrt(np.mean(np.sum((img - projected.reshape(-1, 2)) ** 2, axis=1))))
            if not math.isfinite(rms) or rms > 5.0:
                continue
            T_cam_opt_map = np.eye(4, dtype=float)
            T_cam_opt_map[:3, :3], _ = cv2.Rodrigues(rvec)
            T_cam_opt_map[:3, 3] = np.asarray(tvec).reshape(3)
            T_map_cam_opt = invert_transform(T_cam_opt_map)
            T_map_cam_ros = T_map_cam_opt @ ros_to_optical_matrix()
            candidates.append(invert_transform(T_map_base) @ T_map_cam_ros)
            reprojection.append(rms)
            used_tag_ids.update(frame_ids)
        except (ValueError, cv2.error, np.linalg.LinAlgError):
            continue

    if len(candidates) < int(min_frames):
        raise ValueError(
            f'Need at least {int(min_frames)} frames containing {int(min_tags)} '
            f'confirmed visible tags; collected {len(candidates)}. '
            'Manual camera coordinates remain available.'
        )
    result, inliers, t_error, r_error = _robust_transform_average(candidates, min_frames)
    mount = validate_camera_mount(result)
    reproj = np.asarray(reprojection, dtype=float)[inliers]
    diagnostics = {
        'samples': len(candidates),
        'inliers': int(np.count_nonzero(inliers)),
        'tags_used': sorted(used_tag_ids),
        'translation_spread_mm': round(float(np.sqrt(np.mean(t_error ** 2))) * 1000.0, 2),
        'rotation_spread_deg': round(math.degrees(float(np.sqrt(np.mean(r_error ** 2)))), 2),
        'reprojection_rms_px': round(float(np.sqrt(np.mean(reproj ** 2))), 2),
        **mount,
    }
    return result, diagnostics

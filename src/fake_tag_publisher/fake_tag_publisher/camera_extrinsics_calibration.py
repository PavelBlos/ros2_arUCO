"""Robust hand-eye estimate for a stationary robot under a mapped marker."""

import math
import numpy as np
from scipy.spatial.transform import Rotation as Rotation

try:
    from .geometry_transforms import invert_transform
except ImportError:
    from geometry_transforms import invert_transform


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

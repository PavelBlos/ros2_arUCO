"""Gravity-constrained localization for horizontal ceiling markers.

Undistorted camera rays are rotated into the level robot frame, then a
positive 2-D similarity fits their intersection with the ceiling to the map.
The fitted scale is the camera/ceiling separation, not an arbitrary map scale.
No single-marker out-of-plane IPPE rotation enters the robot pose.
"""
import math
import cv2
import numpy as np


# A 100 mm marker is only about 35-40 pixels wide at the measured ceiling
# height. Its independently learned map pose can therefore carry roughly a
# centimetre of corner-equivalent error even when the two per-marker robot
# poses agree. Keep the strict per-marker fit, then allow that measured map
# uncertainty in the joint fit. The independent 8 cm / 5 degree pose gate
# below still rejects a genuinely inconsistent map before this is used.
SINGLE_TAG_MAX_REPROJ_PX = 2.5
JOINT_TAG_MAX_REPROJ_PX = 5.0

try:
    from .geometry_transforms import optical_to_ros_rotation
    from .single_tag_pnp import get_marker_object_points
except ImportError:
    from geometry_transforms import optical_to_ros_rotation
    from single_tag_pnp import get_marker_object_points


def rotation2(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def ceiling_rays(corners, K, distortion, T_base_cam):
    pixels = np.asarray(corners, dtype=float).reshape(-1, 2)
    if len(pixels) != 4 or not np.all(np.isfinite(pixels)):
        raise ValueError('Four finite corners are required')
    normalized = cv2.undistortPoints(pixels.reshape(-1, 1, 2), K, distortion).reshape(-1, 2)
    optical = np.column_stack([normalized, np.ones(4)])
    rays = optical @ (T_base_cam[:3, :3] @ optical_to_ros_rotation()).T
    if np.any(rays[:, 2] <= 0.15):
        raise ValueError('Marker rays do not intersect the ceiling above the robot')
    return rays[:, :2] / rays[:, 2, None]


def fit_similarity(source, target, weights=None):
    source, target = np.asarray(source), np.asarray(target)
    w = np.ones(len(source)) if weights is None else np.asarray(weights)
    w = w / w.sum()
    a, b = np.sum(source*w[:, None], axis=0), np.sum(target*w[:, None], axis=0)
    x, y = source-a, target-b
    u, _, vt = np.linalg.svd((x*w[:, None]).T @ y)
    correction = np.diag([1., np.linalg.det(vt.T @ u.T)])
    rot = vt.T @ correction @ u.T
    denom = np.sum(w * np.sum(x*x, axis=1))
    if denom < 1e-12:
        raise ValueError('Degenerate ceiling observation')
    scale = float(np.sum(w*np.sum((x@rot.T)*y, axis=1))/denom)
    if not 0.15 < scale < 10.0:
        raise ValueError('Implausible ceiling distance')
    return scale, rot, b-scale*(rot@a)


def marker_map_corners(info, size_m):
    p = info['pose']
    # Downward normal: roll=pi maps local marker Y to -map Y.
    normal = np.array([math.sin(p['yaw'])*math.sin(p['roll']) + math.cos(p['yaw'])*math.sin(p['pitch'])*math.cos(p['roll']),
                       -math.cos(p['yaw'])*math.sin(p['roll']) + math.sin(p['yaw'])*math.sin(p['pitch'])*math.cos(p['roll']),
                       math.cos(p['pitch'])*math.cos(p['roll'])])
    if np.linalg.norm(normal-np.array([0., 0., -1.])) > 0.035:
        raise ValueError('Ceiling map contains a tilted marker; rebuild its geometry')
    local = get_marker_object_points(size_m)[:, :2] * [1., -1.]
    return local @ rotation2(p['yaw']).T + [p['x'], p['y']]


def project_ceiling(points_map, height, rot, camera_xy, K, distortion, T_base_cam):
    body = np.column_stack([(np.asarray(points_map)-camera_xy) @ rot, np.full(len(points_map), height)])
    optical = body @ (T_base_cam[:3, :3] @ optical_to_ros_rotation())
    if np.any(optical[:, 2] <= 0):
        raise ValueError('Projection behind camera')
    return cv2.projectPoints(optical, np.zeros(3), np.zeros(3), K, distortion)[0].reshape(-1, 2)


def solve_ceiling_frame(detections, tags, K, distortion, T_base_cam):
    candidates = []
    rejected = {}
    for d in detections:
        tid = str(d['tag_id'])
        info = tags.get(tid)
        if not info or not info.get('enabled', True) or info.get('state') != 'confirmed':
            continue
        try:
            pixels = np.asarray(d['corners_px']).reshape(4, 2)
            rays = ceiling_rays(pixels, K, distortion, T_base_cam)
            points = marker_map_corners(info, float(info.get('size_mm', d.get('marker_size_mm', 100.)))/1000.)
            h, rot, camera_xy = fit_similarity(rays, points)
            err = float(np.sqrt(np.mean(np.sum((project_ceiling(points, h, rot, camera_xy, K, distortion, T_base_cam)-pixels)**2, axis=1))))
            if err > SINGLE_TAG_MAX_REPROJ_PX:
                raise ValueError(
                    f'Ceiling reprojection residual exceeds {SINGLE_TAG_MAX_REPROJ_PX:.1f} px'
                )
            base_xy = camera_xy - rot @ T_base_cam[:2, 3]
            candidates.append(dict(id=tid, rays=rays, points=points, pixels=pixels,
                                   height=h, rot=rot, camera_xy=camera_xy, base_xy=base_xy, error=err))
        except (ValueError, KeyError, np.linalg.LinAlgError, cv2.error) as e:
            rejected[tid] = str(e)
    if not candidates:
        return dict(status='ceiling_no_valid_tags', fused_base_pose=None, inlier_ids=[],
                    rejected_ids=list(rejected), rejection_reasons=rejected, multi_tag_used=False, reproj_rms_px=999.)
    # Do not average an inconsistent map or silently choose one of two tags.
    for i, a in enumerate(candidates):
        for b in candidates[i+1:]:
            dyaw = math.atan2((a['rot'].T@b['rot'])[1, 0], (a['rot'].T@b['rot'])[0, 0])
            if np.linalg.norm(a['base_xy']-b['base_xy']) > .08 or abs(dyaw) > math.radians(5):
                return dict(status='multi_tag_conflict', fused_base_pose=None, inlier_ids=[],
                            rejected_ids=[a['id'], b['id']], rejection_reasons={a['id']:'ceiling_map_inconsistent', b['id']:'ceiling_map_inconsistent'},
                            multi_tag_used=False, reproj_rms_px=999.)
    rays = np.vstack([c['rays'] for c in candidates])
    points = np.vstack([c['points'] for c in candidates])
    weights = np.repeat([1./max(.3, c['error'])**2 for c in candidates], 4)
    height, rot, camera_xy = fit_similarity(rays, points, weights)
    per_tag = {}
    for c in candidates:
        projected = project_ceiling(c['points'], height, rot, camera_xy, K, distortion, T_base_cam)
        per_tag[c['id']] = float(np.sqrt(np.mean(np.sum((projected-c['pixels'])**2, axis=1))))
    if max(per_tag.values()) > JOINT_TAG_MAX_REPROJ_PX:
        return dict(status='multi_tag_conflict', fused_base_pose=None, inlier_ids=[],
                    rejected_ids=list(per_tag), rejection_reasons={k:'joint_ceiling_residual' for k in per_tag},
                    multi_tag_used=False, reproj_rms_px=max(per_tag.values()),
                    per_tag_residuals=per_tag)
    xy = camera_xy-rot@T_base_cam[:2, 3]
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    return dict(status='multi_tag_ok' if len(candidates)>1 else 'single_tag_ok',
                fused_base_pose=(float(xy[0]), float(xy[1]), yaw), inlier_ids=[c['id'] for c in candidates],
                rejected_ids=list(rejected), rejection_reasons=rejected,
                reproj_rms_px=float(np.sqrt(np.mean(np.square(list(per_tag.values()))))),
                multi_tag_used=len(candidates)>1, covariance=np.diag([.015**2, .015**2, math.radians(1)**2]),
                ceiling_height_m=height, per_tag_residuals=per_tag)


def locate_ceiling_tag(detection, size_m, robot_pose, K, distortion, T_base_cam):
    """Learn XY/yaw and height with gravity fixed, using an independent robot pose."""
    rays = ceiling_rays(detection['corners_px'], K, distortion, T_base_cam)
    local = get_marker_object_points(size_m)[:, :2]*[1., -1.]
    height, tag_from_body, offset = fit_similarity(rays, local)
    # local = H * R_tag_body * ray + offset, so target center in body = -R.T*offset.
    body_center = -tag_from_body.T @ offset + T_base_cam[:2, 3]
    rot = rotation2(robot_pose[2])
    xy = np.asarray(robot_pose[:2]) + rot@body_center
    tag_rot = rot @ tag_from_body.T
    return dict(x=float(xy[0]), y=float(xy[1]), z=float(height+T_base_cam[2, 3]),
                roll=math.pi, pitch=0., yaw=math.atan2(tag_rot[1,0], tag_rot[0,0]))


def vertical_target_pixel(K, distortion, T_base_cam, ceiling_height_m):
    """Pixel of a ceiling point vertically above the base origin."""
    point_body = np.array([0., 0., ceiling_height_m])-T_base_cam[:3, 3]
    optical = optical_to_ros_rotation().T @ T_base_cam[:3, :3].T @ point_body
    if optical[2] <= 0:
        raise ValueError('Camera does not face the ceiling')
    return cv2.projectPoints(optical.reshape(1,3), np.zeros(3), np.zeros(3), K, distortion)[0].reshape(2)

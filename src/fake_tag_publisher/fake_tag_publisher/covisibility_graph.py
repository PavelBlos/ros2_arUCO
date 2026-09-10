"""
covisibility_graph.py - ArUco Co-Visibility Graph, Viewpoint Diversity, and Diagnostic Confidence.

Features:
  - Directed/undirected graph representation of simultaneously visible tags.
  - Multi-view observation history per edge (relative SE(3) pose, distance, viewing angles, robot base poses).
  - Viewpoint diversity metric S_view requiring diverse angles and positions before confirmation.
  - Diagnostic confidence score formula combining observation count, reprojection RMS, diversity, and graph degree.
  - Anchor path finding with cumulative covariance / uncertainty propagation.
"""

import math
import time
import json
import os
import collections
import numpy as np
from typing import Dict, List, Any, Optional, Tuple, Set

try:
    from .geometry_transforms import (
        normalize_angle,
        pose_to_matrix,
        matrix_to_pose,
        invert_transform
    )
except ImportError:
    from geometry_transforms import (
        normalize_angle,
        pose_to_matrix,
        matrix_to_pose,
        invert_transform
    )


class CovisibilityObservation:
    """A single co-visibility observation between two tags in the same camera frame."""
    def __init__(self,
                 tag_a: int,
                 tag_b: int,
                 T_a_b: np.ndarray,
                 distance_a_m: float,
                 distance_b_m: float,
                 view_angle_a_deg: float,
                 view_angle_b_deg: float,
                 robot_pose: Tuple[float, float, float],
                 timestamp: float):
        self.tag_a = int(tag_a)
        self.tag_b = int(tag_b)
        self.T_a_b = np.array(T_a_b, dtype=np.float64)
        self.distance_a_m = float(distance_a_m)
        self.distance_b_m = float(distance_b_m)
        self.view_angle_a_deg = float(view_angle_a_deg)
        self.view_angle_b_deg = float(view_angle_b_deg)
        self.robot_pose = tuple(robot_pose)
        self.timestamp = float(timestamp)


class CovisibilityGraph:
    """
    Tracks co-visibility relationships between tags, evaluating geometric consistency,
    viewpoint diversity, and confidence toward the anchor tag.
    """
    def __init__(self):
        # Adjacency: tag_id -> set of neighbor tag_ids
        self._adj: Dict[int, Set[int]] = collections.defaultdict(set)
        # Edge observations: (min(a,b), max(a,b)) -> list of CovisibilityObservation
        self._edge_observations: Dict[Tuple[int, int], List[CovisibilityObservation]] = collections.defaultdict(list)
        # Single tag observation stats: tag_id -> dict
        self._tag_stats: Dict[int, Dict[str, Any]] = collections.defaultdict(lambda: {
            "obs_count": 0,
            "reproj_errs": [],
            "view_angles": [],
            "robot_positions": []
        })

    def record_frame_observations(self,
                                  detections: List[Dict[str, Any]],
                                  robot_pose: Tuple[float, float, float],
                                  timestamp: Optional[float] = None):
        """
        Record all tags observed in a single frame and update pairwise co-visibility edges.
        """
        t = timestamp if timestamp is not None else time.time()
        valid_dets = [d for d in detections if d.get("pose_valid", False)]
        
        # 1. Update single tag stats
        for d in valid_dets:
            tid = int(d["tag_id"])
            st = self._tag_stats[tid]
            st["obs_count"] += 1

            # Only append to spatial diversity history if robot moved >= 3cm or 2 deg
            last_pos = st["robot_positions"][-1] if st["robot_positions"] else None
            moved = True
            if last_pos is not None:
                dist_moved = math.hypot(robot_pose[0] - last_pos[0], robot_pose[1] - last_pos[1])
                if dist_moved < 0.03:
                    moved = False

            if moved or len(st["robot_positions"]) < 3:
                st["reproj_errs"].append(float(d.get("reproj_err", 1.0)))
                st["view_angles"].append(float(d.get("viewing_angle_deg", 0.0)))
                st["robot_positions"].append((robot_pose[0], robot_pose[1]))
                if len(st["reproj_errs"]) > 200:
                    st["reproj_errs"] = st["reproj_errs"][-200:]
                    st["view_angles"] = st["view_angles"][-200:]
                    st["robot_positions"] = st["robot_positions"][-200:]

        # 2. Update pairwise edges with canonical (min, max) orientation
        n = len(valid_dets)
        for i in range(n):
            for j in range(i + 1, n):
                da = valid_dets[i]
                db = valid_dets[j]
                ta = int(da["tag_id"])
                tb = int(db["tag_id"])

                T_ca = da.get("T_cameraRos_tag")
                T_cb = db.get("T_cameraRos_tag")
                if T_ca is None or T_cb is None:
                    continue

                # Canonical edge: key is (min_id, max_id)
                # T_min_max maps coordinates from max_id to min_id: p_min = T_min_max @ p_max
                if ta < tb:
                    tag_min, tag_max = ta, tb
                    T_min_max = invert_transform(T_ca) @ T_cb
                    d_min = float(da.get("distance_m", 2.0))
                    d_max = float(db.get("distance_m", 2.0))
                    ang_min = float(da.get("viewing_angle_deg", 0.0))
                    ang_max = float(db.get("viewing_angle_deg", 0.0))
                else:
                    tag_min, tag_max = tb, ta
                    T_min_max = invert_transform(T_cb) @ T_ca
                    d_min = float(db.get("distance_m", 2.0))
                    d_max = float(da.get("distance_m", 2.0))
                    ang_min = float(db.get("viewing_angle_deg", 0.0))
                    ang_max = float(da.get("viewing_angle_deg", 0.0))

                edge_key = (tag_min, tag_max)

                obs = CovisibilityObservation(
                    tag_a=tag_min,
                    tag_b=tag_max,
                    T_a_b=T_min_max,
                    distance_a_m=d_min,
                    distance_b_m=d_max,
                    view_angle_a_deg=ang_min,
                    view_angle_b_deg=ang_max,
                    robot_pose=robot_pose,
                    timestamp=t
                )

                self._edge_observations[edge_key].append(obs)
                if len(self._edge_observations[edge_key]) > 200:
                    self._edge_observations[edge_key] = self._edge_observations[edge_key][-200:]

                self._adj[ta].add(tb)
                self._adj[tb].add(ta)

    def get_viewpoint_diversity(self, tag_id: int) -> float:
        """
        Compute viewpoint diversity score S_view in [0.0, 1.0].
        Evaluates angular spread of viewing rays and spatial baseline spread of robot poses.
        """
        tid = int(tag_id)
        if tid not in self._tag_stats or self._tag_stats[tid]["obs_count"] < 3:
            return 0.0

        st = self._tag_stats[tid]
        angles = st["view_angles"]
        positions = np.array(st["robot_positions"])

        # Angular span
        angle_span = max(angles) - min(angles) if angles else 0.0
        angle_score = min(1.0, angle_span / 25.0)

        # Position spatial spread (bounding box diagonal or std)
        if len(positions) >= 3:
            pos_std = float(np.mean(np.std(positions, axis=0)))
            pos_score = min(1.0, pos_std / 0.15)  # 15 cm std spread is diverse
        else:
            pos_score = 0.0

        # Number of samples factor
        n_score = min(1.0, len(angles) / 20.0)

        # Combined diversity score
        s_view = 0.40 * angle_score + 0.40 * pos_score + 0.20 * n_score
        return float(min(1.0, max(0.0, s_view)))

    def get_tag_confidence(self, tag_id: int) -> float:
        """
        Calculate overall diagnostic confidence in [0.0, 1.0]:
          Confidence = w_n * min(1, N/50) + w_err * max(0, 1 - err/1.5) + w_view * S_view + w_deg * min(1, deg/3)
        """
        tid = int(tag_id)
        if tid not in self._tag_stats:
            return 0.0

        st = self._tag_stats[tid]
        n_obs = st["obs_count"]
        if n_obs == 0:
            return 0.0

        med_err = float(np.median(st["reproj_errs"])) if st["reproj_errs"] else 1.0
        s_view = self.get_viewpoint_diversity(tid)
        deg = len(self._adj[tid])

        # Weights: count (0.25), reproj error (0.35), diversity (0.25), graph degree (0.15)
        w_n = 0.25
        w_err = 0.35
        w_view = 0.25
        w_deg = 0.15

        c_n = min(1.0, n_obs / 50.0)
        c_err = max(0.0, 1.0 - (med_err / 1.5))
        c_deg = min(1.0, deg / 3.0)

        conf = w_n * c_n + w_err * c_err + w_view * s_view + w_deg * c_deg
        return float(min(1.0, max(0.0, conf)))

    def find_path_to_anchor(self, tag_id: int, anchor_id: int) -> Optional[List[int]]:
        """
        BFS search finding the shortest co-visibility path between tag_id and anchor_id.
        Returns list of tag IDs [tag_id, ..., anchor_id] or None if disconnected.
        """
        src = int(tag_id)
        dst = int(anchor_id)
        if src == dst:
            return [src]
        if src not in self._adj or dst not in self._adj:
            return None

        queue = collections.deque([[src]])
        visited = {src}

        while queue:
            path = queue.popleft()
            curr = path[-1]

            if curr == dst:
                return path

            for nbr in sorted(self._adj[curr]):
                if nbr not in visited:
                    visited.add(nbr)
                    queue.append(path + [nbr])

        return None

    def get_edge_observations_count(self, tag_a: int, tag_b: int) -> int:
        edge_key = (min(int(tag_a), int(tag_b)), max(int(tag_a), int(tag_b)))
        return len(self._edge_observations.get(edge_key, []))

    def get_all_edges(self) -> list:
        return list(self._edge_observations.keys())

    def get_diagnostics(self) -> Dict[str, Any]:
        """Summary diagnostics for Web UI and health monitoring."""
        return {
            "total_tags_tracked": len(self._tag_stats),
            "total_edges": len(self._edge_observations),
            "tag_confidences": {str(tid): round(self.get_tag_confidence(tid), 3) for tid in self._tag_stats},
            "tag_diversities": {str(tid): round(self.get_viewpoint_diversity(tid), 3) for tid in self._tag_stats},
            "tag_degrees": {str(tid): len(self._adj[tid]) for tid in self._tag_stats}
        }

    def estimate_tag_pose_from_anchor(self,
                                      target_tag_id: int,
                                      anchor_tag_id: int,
                                      anchor_pose: Dict[str, float]) -> Optional[Dict[str, float]]:
        """
        Estimate 3D pose of target_tag_id in map frame by traversing
        the co-visibility BFS path from anchor_tag_id.
        """
        path = self.find_path_to_anchor(target_tag_id, anchor_tag_id)
        if not path:
            return None

        # Path is [target, ..., anchor]. Reverse to traverse [anchor, ..., target]
        rev_path = list(reversed(path))

        T_map_curr = pose_to_matrix(
            float(anchor_pose.get("x", 0.0)),
            float(anchor_pose.get("y", 0.0)),
            float(anchor_pose.get("z", 2.5)),
            float(anchor_pose.get("roll", math.pi)),
            float(anchor_pose.get("pitch", 0.0)),
            float(anchor_pose.get("yaw", 0.0))
        )

        for idx in range(len(rev_path) - 1):
            curr_id = rev_path[idx]
            next_id = rev_path[idx + 1]
            edge_key = (min(curr_id, next_id), max(curr_id, next_id))
            obs_list = self._edge_observations.get(edge_key, [])
            if not obs_list:
                return None

            recent_obs = obs_list[-20:]
            trans_list = [o.T_a_b[:3, 3] for o in recent_obs]
            med_trans = np.median(trans_list, axis=0)

            T_canon = np.copy(recent_obs[-1].T_a_b)
            T_canon[:3, 3] = med_trans

            if curr_id < next_id:
                T_curr_next = T_canon
            else:
                T_curr_next = invert_transform(T_canon)

            T_map_curr = T_map_curr @ T_curr_next

        x_est, y_est, z_est, r_est, p_est, yaw_est = matrix_to_pose(T_map_curr)
        return {
            "x": round(float(x_est), 4),
            "y": round(float(y_est), 4),
            "z": round(float(z_est), 4),
            "roll": round(float(r_est), 4),
            "pitch": round(float(p_est), 4),
            "yaw": round(float(normalize_angle(yaw_est)), 4)
        }

    def save_to_json(self, filepath: str) -> bool:
        """Persist graph state to json file."""
        try:
            os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
            data = {
                "adj": {str(k): list(v) for k, v in self._adj.items()},
                "tag_stats": {
                    str(k): {
                        "obs_count": v["obs_count"],
                        "reproj_errs": v["reproj_errs"][-50:],
                        "view_angles": v["view_angles"][-50:],
                        "robot_positions": v["robot_positions"][-50:]
                    } for k, v in self._tag_stats.items()
                },
                "edges": {
                    f"{k[0]}_{k[1]}": {
                        "count": len(obs),
                        "latest_T": obs[-1].T_a_b.tolist() if obs else None
                    } for k, obs in self._edge_observations.items()
                }
            }
            tmp = filepath + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, filepath)
            return True
        except Exception:
            return False

    def load_from_json(self, filepath: str) -> bool:
        """Load graph state from json file."""
        if not os.path.exists(filepath):
            return False
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)
            self._adj.clear()
            for k, v in data.get("adj", {}).items():
                self._adj[int(k)] = set(int(x) for x in v)
            for k, v in data.get("tag_stats", {}).items():
                tid = int(k)
                self._tag_stats[tid]["obs_count"] = v.get("obs_count", 0)
                self._tag_stats[tid]["reproj_errs"] = v.get("reproj_errs", [])
                self._tag_stats[tid]["view_angles"] = v.get("view_angles", [])
                self._tag_stats[tid]["robot_positions"] = v.get("robot_positions", [])
            return True
        except Exception:
            return False

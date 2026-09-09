"""
tag_registry.py - Unified, Thread-Safe, Versioned Tag Registry (Schema v2).

Features:
  - Canonical Schema v2 management.
  - Thread safety via threading.RLock.
  - Atomic persistence (write .tmp -> fsync -> os.replace).
  - Timestamped backups and append-only audit log.
  - Migration from legacy tags_config.yaml (preserves tags as unconfirmed, anchor unconfirmed).
  - Optimistic concurrency control (HTTP 409 Conflict if expected_revision != current_revision).
  - Validation: Dictionary ID range (0-99 for DICT_4X4_100), math.isfinite, positive size/z.
  - Protection of anchor tag from deletion or disabling without re-anchoring.
  - Full SHA-256 calculation for file and data.
"""

import os
import sys
import time
import math
import yaml
import hashlib
import threading
from typing import Dict, Any, Optional, Tuple

class RegistryError(Exception):
    """Base exception for registry errors."""
    pass

class RevisionConflictError(RegistryError):
    """Raised when an update conflicts with the current revision (optimistic locking)."""
    pass

class ValidationError(RegistryError):
    """Raised when data fails schema, type, range, or domain validation."""
    pass

class AnchorProtectionError(RegistryError):
    """Raised when attempting to delete or disable an active anchor tag."""
    pass

VALID_STATES = {"confirmed", "provisional", "unconfirmed", "disabled"}
SUPPORTED_DICTIONARIES = {
    "DICT_4X4_50": 50,
    "DICT_4X4_100": 100,
    "DICT_4X4_250": 250,
    "DICT_4X4_1000": 1000,
    "DICT_5X5_50": 50,
    "DICT_5X5_100": 100,
    "DICT_5X5_250": 250,
    "DICT_5X5_1000": 1000,
    "DICT_6X6_50": 50,
    "DICT_6X6_100": 100,
    "DICT_6X6_250": 250,
    "DICT_6X6_1000": 1000,
    "DICT_7X7_50": 50,
    "DICT_7X7_100": 100,
    "DICT_7X7_250": 250,
    "DICT_7X7_1000": 1000,
}

def calc_sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()

def calc_sha256_file(filepath: str) -> str:
    h = hashlib.sha256()
    with open(filepath, "rb") as f:
        while chunk := f.read(8192):
            h.update(chunk)
    return h.hexdigest()

class TagRegistry:
    def __init__(self, filepath: str, backup_dir: Optional[str] = None):
        self.filepath = os.path.abspath(filepath)
        self.lock = threading.RLock()
        
        if backup_dir:
            self.backup_dir = os.path.abspath(backup_dir)
        else:
            self.backup_dir = os.path.join(os.path.dirname(self.filepath), "backups")
        
        self.audit_log_path = os.path.join(os.path.dirname(self.filepath), "tag_map_audit.log")
        
        self._data: Dict[str, Any] = {}
        self._sha256: str = ""
        
        with self.lock:
            if os.path.exists(self.filepath):
                self.load()
            else:
                self._create_default()

    def _create_default(self):
        """Create empty default Schema v2 document."""
        self._data = {
            "schema_version": 2,
            "config_epoch": 1,
            "revision": 1,
            "dictionary": "DICT_4X4_100",
            "default_marker_size_mm": 100.0,
            "ceiling_z_m": 2.5,
            "anchor_tag_id": None,
            "tags": {}
        }
        self.save(comment="Initial default schema v2 creation")

    def _log_audit(self, action: str, details: str):
        try:
            os.makedirs(os.path.dirname(self.audit_log_path), exist_ok=True)
            iso_now = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            line = f"[{iso_now}] [REV:{self.revision}] [{action}] {details}\n"
            with open(self.audit_log_path, "a", encoding="utf-8") as f:
                f.write(line)
        except Exception:
            pass

    @property
    def revision(self) -> int:
        with self.lock:
            return int(self._data.get("revision", 1))

    @property
    def config_epoch(self) -> int:
        with self.lock:
            return int(self._data.get("config_epoch", 1))

    @property
    def sha256(self) -> str:
        with self.lock:
            return self._sha256

    @property
    def dictionary_name(self) -> str:
        with self.lock:
            return str(self._data.get("dictionary", "DICT_4X4_100"))

    @property
    def default_marker_size_mm(self) -> float:
        with self.lock:
            return float(self._data.get("default_marker_size_mm", 100.0))

    @property
    def anchor_tag_id(self) -> Optional[str]:
        with self.lock:
            return self._data.get("anchor_tag_id")

    def get_tag(self, tag_id: int | str) -> Optional[Dict[str, Any]]:
        norm_id = str(int(str(tag_id).replace("tag_", "")))
        with self.lock:
            tag_entry = self._data.get("tags", {}).get(norm_id)
            if tag_entry:
                import copy
                return copy.deepcopy(tag_entry)
            return None

    def get_marker_size_m(self, tag_id: int | str) -> float:
        """Return marker side length in meters (checking tag override, else default)."""
        tag = self.get_tag(tag_id)
        if tag and "size_mm" in tag and tag["size_mm"] is not None:
            return float(tag["size_mm"]) / 1000.0
        return self.default_marker_size_mm / 1000.0

    def get_all_tags(self) -> Dict[str, Any]:
        with self.lock:
            import copy
            return copy.deepcopy(self._data.get("tags", {}))

    def get_active_tags_for_localization(self) -> Dict[str, Any]:
        """Return only tags that are strictly confirmed AND enabled."""
        with self.lock:
            res = {}
            for tid, tag in self._data.get("tags", {}).items():
                if tag.get("enabled", False) and tag.get("state") == "confirmed":
                    res[tid] = tag
            import copy
            return copy.deepcopy(res)

    def validate_tag_data(self, tag_id: int | str, data: Dict[str, Any]):
        try:
            num_id = int(str(tag_id).replace("tag_", ""))
        except (ValueError, TypeError):
            raise ValidationError(f"Invalid tag ID: {tag_id}. Must be numeric.")
        
        max_id = SUPPORTED_DICTIONARIES.get(self.dictionary_name, 100)
        if num_id < 0 or num_id >= max_id:
            raise ValidationError(f"Tag ID {num_id} out of bounds for dictionary {self.dictionary_name} [0, {max_id - 1}].")
        
        state = data.get("state", "unconfirmed")
        if state not in VALID_STATES:
            raise ValidationError(f"Invalid state: {state}. Must be one of {VALID_STATES}.")
        
        pose = data.get("pose", {})
        for k in ["x", "y", "z", "roll", "pitch", "yaw"]:
            if k not in pose:
                raise ValidationError(f"Missing pose field: {k}")
            val = pose[k]
            if not isinstance(val, (int, float)) or not math.isfinite(val):
                raise ValidationError(f"Pose coordinate {k}={val} must be a finite number.")
        
        if pose["z"] <= 0:
            raise ValidationError(f"Ceiling z={pose['z']} must be positive.")
        
        if "size_mm" in data and data["size_mm"] is not None:
            sz = data["size_mm"]
            if not isinstance(sz, (int, float)) or not math.isfinite(sz) or sz < 20.0 or sz > 1000.0:
                raise ValidationError(f"Marker size_mm={sz} must be between 20 mm and 1000 mm.")

    def set_tag(self, tag_id: int | str, data: Dict[str, Any], expected_revision: Optional[int] = None) -> Tuple[int, str]:
        with self.lock:
            if expected_revision is not None and expected_revision != self.revision:
                raise RevisionConflictError(
                    f"Revision conflict: current revision is {self.revision}, but expected {expected_revision}."
                )
            
            self.validate_tag_data(tag_id, data)
            norm_id = str(int(str(tag_id).replace("tag_", "")))
            
            # Check anchor protection
            if norm_id == self.anchor_tag_id:
                if not data.get("enabled", True):
                    raise AnchorProtectionError(f"Cannot disable anchor tag {norm_id}. Assign a new anchor first.")
                if data.get("state") != "confirmed":
                    raise AnchorProtectionError(f"Cannot demote anchor tag {norm_id} from confirmed state.")

            tag_entry = {
                "enabled": bool(data.get("enabled", True)),
                "state": data.get("state", "unconfirmed"),
                "pose": {
                    "x": float(data["pose"]["x"]),
                    "y": float(data["pose"]["y"]),
                    "z": float(data["pose"]["z"]),
                    "roll": float(data["pose"]["roll"]),
                    "pitch": float(data["pose"]["pitch"]),
                    "yaw": float(data["pose"]["yaw"]),
                },
                "size_mm": float(data["size_mm"]) if data.get("size_mm") is not None else self.default_marker_size_mm,
                "source": str(data.get("source", "manual")),
                "covariance": data.get("covariance", None),
                "observations": int(data.get("observations", 0)),
                "parent_tags": list(data.get("parent_tags", [])),
                "updated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            }
            
            self._data.setdefault("tags", {})[norm_id] = tag_entry
            self._data["revision"] = self.revision + 1
            self.save(comment=f"Set tag {norm_id} (state={tag_entry['state']}, enabled={tag_entry['enabled']})")
            return self.revision, self.sha256

    def delete_tag(self, tag_id: int | str, expected_revision: Optional[int] = None) -> Tuple[int, str]:
        """Soft-disables tag or removes it if unconfirmed."""
        with self.lock:
            if expected_revision is not None and expected_revision != self.revision:
                raise RevisionConflictError(f"Revision conflict: expected {expected_revision}, got {self.revision}.")
            norm_id = str(int(str(tag_id).replace("tag_", "")))
            
            if norm_id == self.anchor_tag_id:
                raise AnchorProtectionError(f"Cannot delete active anchor tag {norm_id}. Designate another anchor first.")
            
            if norm_id not in self._data.get("tags", {}):
                raise RegistryError(f"Tag {norm_id} not found in registry.")
            
            tag = self._data["tags"][norm_id]
            if tag.get("state") == "confirmed":
                tag["enabled"] = False
                tag["state"] = "disabled"
                tag["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
                action = f"Disabled confirmed tag {norm_id}"
            else:
                del self._data["tags"][norm_id]
                action = f"Deleted unconfirmed tag {norm_id}"
            
            self._data["revision"] = self.revision + 1
            self.save(comment=action)
            return self.revision, self.sha256

    def set_anchor(self, tag_id: int | str, size_mm: float, ceiling_z_m: float,
                   expected_revision: Optional[int] = None) -> Tuple[int, str]:
        """Explicitly designate and verify a tag as map origin (0, 0, z, roll=pi, pitch=0, yaw=0)."""
        with self.lock:
            if expected_revision is not None and expected_revision != self.revision:
                raise RevisionConflictError(f"Revision conflict: expected {expected_revision}, got {self.revision}.")
            norm_id = str(int(str(tag_id).replace("tag_", "")))
            
            data = {
                "enabled": True,
                "state": "confirmed",
                "pose": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": float(ceiling_z_m),
                    "roll": float(math.pi),
                    "pitch": 0.0,
                    "yaw": 0.0
                },
                "size_mm": float(size_mm),
                "source": "anchor_wizard"
            }
            self.validate_tag_data(norm_id, data)
            self._data.setdefault("tags", {})[norm_id] = {
                **data,
                "covariance": None,
                "observations": 0,
                "parent_tags": [],
                "updated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            }
            self._data["anchor_tag_id"] = norm_id
            self._data["ceiling_z_m"] = float(ceiling_z_m)
            self._data["revision"] = self.revision + 1
            self.save(comment=f"Designated tag {norm_id} as confirmed anchor (origin)")
            return self.revision, self.sha256

    def set_settings(self, dictionary: Optional[str] = None, default_marker_size_mm: Optional[float] = None,
                     expected_revision: Optional[int] = None) -> Tuple[int, str]:
        with self.lock:
            if expected_revision is not None and expected_revision != self.revision:
                raise RevisionConflictError(f"Revision conflict: expected {expected_revision}, got {self.revision}.")
            
            mutated = False
            if dictionary is not None:
                if dictionary not in SUPPORTED_DICTIONARIES:
                    raise ValidationError(f"Unsupported dictionary: {dictionary}. Options: {list(SUPPORTED_DICTIONARIES.keys())}")
                self._data["dictionary"] = dictionary
                mutated = True
            
            if default_marker_size_mm is not None:
                sz = float(default_marker_size_mm)
                if sz < 20.0 or sz > 1000.0:
                    raise ValidationError(f"default_marker_size_mm={sz} must be between 20 and 1000 mm.")
                self._data["default_marker_size_mm"] = sz
                mutated = True
            
            if mutated:
                self._data["revision"] = self.revision + 1
                self.save(comment="Updated registry settings (dictionary/marker_size)")
            return self.revision, self.sha256

    def save(self, comment: str = ""):
        """Atomic write: serialize -> tmp file -> fsync -> atomic replace."""
        with self.lock:
            yaml_content = yaml.dump(self._data, sort_keys=False, allow_unicode=True)
            encoded = yaml_content.encode("utf-8")
            new_sha = calc_sha256_bytes(encoded)
            
            dir_name = os.path.dirname(self.filepath)
            os.makedirs(dir_name, exist_ok=True)
            
            # Backup before overwrite
            if os.path.exists(self.filepath) and self._sha256:
                try:
                    os.makedirs(self.backup_dir, exist_ok=True)
                    ts = time.strftime("%Y%m%d_%H%M%S")
                    backup_file = os.path.join(self.backup_dir, f"tag_map_{ts}_rev{self.revision}.yaml")
                    import shutil
                    shutil.copy2(self.filepath, backup_file)
                except Exception as bkp_err:
                    pass
            
            tmp_path = self.filepath + f".tmp.{os.getpid()}"
            with open(tmp_path, "wb") as f:
                f.write(encoded)
                f.flush()
                os.fsync(f.fileno())
            
            os.replace(tmp_path, self.filepath)
            self._sha256 = new_sha
            self._log_audit("SAVE", f"Saved revision {self.revision}, SHA: {self._sha256[:12]} ({comment})")

    def load(self):
        """Load and validate tag_map.yaml from disk."""
        with self.lock:
            with open(self.filepath, "r", encoding="utf-8") as f:
                content = f.read()
            self._data = yaml.safe_load(content) or {}
            self._sha256 = calc_sha256_bytes(content.encode("utf-8"))
            self._log_audit("LOAD", f"Loaded revision {self.revision}, SHA: {self._sha256[:12]}")

    @classmethod
    def migrate_legacy_tags_config(cls, legacy_path: str, target_tag_map_path: str, backup_dir: Optional[str] = None) -> "TagRegistry":
        """
        Migrate legacy tags_config.yaml into Schema v2 tag_map.yaml.
        Critical requirement: All tags (including tag_17) are migrated as unconfirmed and disabled
        until user explicitly runs Anchor Setup Wizard!
        """
        with open(legacy_path, "r", encoding="utf-8") as f:
            legacy_data = yaml.safe_load(f) or {}
        
        legacy_tags = legacy_data.get("tags", {})
        migrated_tags = {}
        
        for key, info in legacy_tags.items():
            norm_id = str(int(str(key).replace("tag_", "")))
            migrated_tags[norm_id] = {
                "enabled": False,            # UNCONFIRMED AND DISABLED
                "state": "unconfirmed",      # MUST BE EXPLICITLY CONFIRMED BY USER
                "pose": {
                    "x": float(info.get("x", 0.0)),
                    "y": float(info.get("y", 0.0)),
                    "z": float(info.get("z", 2.5)),
                    "roll": float(info.get("roll", math.pi)),
                    "pitch": float(info.get("pitch", 0.0)),
                    "yaw": float(info.get("yaw", 0.0)),
                },
                "size_mm": 100.0,
                "source": "legacy_migration",
                "covariance": None,
                "observations": 0,
                "parent_tags": [],
                "updated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            }
        
        # Save backup of legacy file
        if backup_dir:
            os.makedirs(backup_dir, exist_ok=True)
            import shutil
            ts = time.strftime("%Y%m%d_%H%M%S")
            shutil.copy2(legacy_path, os.path.join(backup_dir, f"legacy_tags_config_{ts}.yaml.bak"))
        
        reg = TagRegistry(target_tag_map_path, backup_dir=backup_dir)
        with reg.lock:
            reg._data = {
                "schema_version": 2,
                "config_epoch": 1,
                "revision": 1,
                "dictionary": "DICT_4X4_100",
                "default_marker_size_mm": 100.0,
                "ceiling_z_m": 2.5,
                "anchor_tag_id": None,      # NO AUTOMATIC ANCHOR!
                "tags": migrated_tags
            }
            reg.save(comment=f"Migrated from {legacy_path} ({len(migrated_tags)} tags unconfirmed)")
        return reg

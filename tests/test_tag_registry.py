"""
test_tag_registry.py - Comprehensive unit tests for TagRegistry (Schema v2).
"""

import os
import math
import pytest
import tempfile
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'fake_tag_publisher', 'fake_tag_publisher')))

from tag_registry import (
    TagRegistry,
    RevisionConflictError,
    ValidationError,
    AnchorProtectionError,
    calc_sha256_file
)

def test_create_and_atomic_save():
    with tempfile.TemporaryDirectory() as tmpdir:
        yaml_path = os.path.join(tmpdir, "tag_map.yaml")
        reg = TagRegistry(yaml_path)
        
        assert reg.revision == 1
        assert reg.dictionary_name == "DICT_4X4_100"
        assert reg.default_marker_size_mm == 100.0
        assert os.path.exists(yaml_path)
        assert len(reg.sha256) == 64

def test_set_and_get_tag():
    with tempfile.TemporaryDirectory() as tmpdir:
        yaml_path = os.path.join(tmpdir, "tag_map.yaml")
        reg = TagRegistry(yaml_path)
        
        tag_data = {
            "enabled": True,
            "state": "confirmed",
            "pose": {"x": 1.5, "y": -2.0, "z": 2.5, "roll": math.pi, "pitch": 0.0, "yaw": 0.5},
            "size_mm": 120.0
        }
        
        rev, sha = reg.set_tag(17, tag_data, expected_revision=1)
        assert rev == 2
        assert reg.revision == 2
        
        tag = reg.get_tag(17)
        assert tag["enabled"] is True
        assert tag["state"] == "confirmed"
        assert tag["size_mm"] == 120.0
        assert reg.get_marker_size_m(17) == 0.120
        assert reg.get_marker_size_m(99) == 0.100  # fallback to default

def test_optimistic_locking_conflict():
    with tempfile.TemporaryDirectory() as tmpdir:
        yaml_path = os.path.join(tmpdir, "tag_map.yaml")
        reg = TagRegistry(yaml_path)
        
        tag_data = {
            "enabled": True,
            "state": "unconfirmed",
            "pose": {"x": 0.0, "y": 0.0, "z": 2.5, "roll": math.pi, "pitch": 0.0, "yaw": 0.0}
        }
        reg.set_tag(10, tag_data, expected_revision=1)
        
        # Now revision is 2. Trying with expected_revision=1 must raise RevisionConflictError
        with pytest.raises(RevisionConflictError):
            reg.set_tag(11, tag_data, expected_revision=1)

def test_validation_bounds():
    with tempfile.TemporaryDirectory() as tmpdir:
        yaml_path = os.path.join(tmpdir, "tag_map.yaml")
        reg = TagRegistry(yaml_path)
        
        # Tag 200 out of bounds for DICT_4X4_100 (0-99)
        with pytest.raises(ValidationError):
            reg.set_tag(200, {
                "enabled": True,
                "state": "confirmed",
                "pose": {"x": 0, "y": 0, "z": 2.5, "roll": 0, "pitch": 0, "yaw": 0}
            })
            
        # Non-finite number
        with pytest.raises(ValidationError):
            reg.set_tag(5, {
                "enabled": True,
                "state": "confirmed",
                "pose": {"x": float('nan'), "y": 0, "z": 2.5, "roll": 0, "pitch": 0, "yaw": 0}
            })

def test_anchor_wizard_and_protection():
    with tempfile.TemporaryDirectory() as tmpdir:
        yaml_path = os.path.join(tmpdir, "tag_map.yaml")
        reg = TagRegistry(yaml_path)
        
        rev, sha = reg.set_anchor(17, size_mm=100.0, ceiling_z_m=2.5)
        assert reg.anchor_tag_id == "17"
        
        anchor_tag = reg.get_tag(17)
        assert anchor_tag["state"] == "confirmed"
        assert anchor_tag["enabled"] is True
        assert anchor_tag["pose"]["x"] == 0.0
        assert anchor_tag["pose"]["y"] == 0.0
        
        # Cannot delete active anchor
        with pytest.raises(AnchorProtectionError):
            reg.delete_tag(17)
            
        # Cannot disable active anchor
        with pytest.raises(AnchorProtectionError):
            reg.set_tag(17, {
                "enabled": False,
                "state": "confirmed",
                "pose": anchor_tag["pose"]
            })

def test_legacy_migration_without_auto_anchor():
    """
    CRITICAL REQUIREMENT:
    Migration preserves legacy tag data, but does NOT assign tag_17 as confirmed anchor!
    It must be unconfirmed until user runs Anchor Wizard.
    """
    with tempfile.TemporaryDirectory() as tmpdir:
        legacy_yaml = os.path.join(tmpdir, "tags_config.yaml")
        with open(legacy_yaml, "w", encoding="utf-8") as f:
            f.write("""tags:
  tag_17:
    enabled: true
    x: 0.0
    y: 0.0
    z: 2.5
    roll: 3.1415926
    pitch: 0.0
    yaw: 0.0
  tag_67:
    enabled: false
    x: 1.0
    y: 2.0
    z: 2.5
    roll: 3.1415926
    pitch: 0.0
    yaw: 0.0
""")
        target_tag_map = os.path.join(tmpdir, "tag_map.yaml")
        reg = TagRegistry.migrate_legacy_tags_config(legacy_yaml, target_tag_map)
        
        assert reg.anchor_tag_id is None  # NO AUTO ANCHOR!
        t17 = reg.get_tag(17)
        assert t17 is not None
        assert t17["state"] == "unconfirmed"  # UNCONFIRMED!
        assert t17["enabled"] is False        # DISABLED!
        
        # Localization active tags must be EMPTY because none are confirmed
        active = reg.get_active_tags_for_localization()
        assert len(active) == 0

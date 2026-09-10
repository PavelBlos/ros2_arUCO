"""
test_migrate_tag_config.py - Unit tests for CLI migration utility.
"""

import os
import sys
import yaml
import pytest
import subprocess
from pathlib import Path

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)
SRC_DIR = os.path.join(PROJECT_DIR, 'src', 'fake_tag_publisher', 'fake_tag_publisher')
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from tag_registry import TagRegistry
from migrate_tag_config import migrate_dict, load_yaml, save_yaml

def test_migrate_dict_unconfirmed_anchor():
    legacy = {
        'default_marker_size_mm': 100.0,
        'ceiling_z_m': 2.5,
        'tag_17': {
            'enabled': True,
            'marker_size_m': 0.100,
            'pose': {'x': 0.0, 'y': 0.0, 'z': 2.5, 'roll': 3.1416, 'pitch': 0.0, 'yaw': 0.0}
        },
        'tag_25': {
            'enabled': False,
            'size_mm': 120.0,
            'pose': {'x': 1.0, 'y': 2.0, 'z': 2.5, 'roll': 3.1416, 'pitch': 0.0, 'yaw': 0.5}
        }
    }

    v2 = migrate_dict(legacy)
    assert v2['schema_version'] == 2
    assert v2['anchor_tag_id'] is None
    assert '17' in v2['tags']
    assert '25' in v2['tags']
    assert v2['tags']['17']['state'] == 'unconfirmed'
    assert v2['tags']['17']['enabled'] is True
    assert v2['tags']['17']['size_mm'] == 100.0
    assert v2['tags']['25']['size_mm'] == 120.0
    assert v2['tags']['25']['enabled'] is False

def test_cli_apply_and_rollback(tmp_path):
    legacy_file = tmp_path / "tags_config.yaml"
    legacy_content = {
        'tag_17': {'enabled': True, 'pose': {'x': 0.0, 'y': 0.0, 'z': 2.5, 'roll': 3.1416, 'pitch': 0.0, 'yaw': 0.0}}
    }
    with open(legacy_file, 'w', encoding='utf-8') as f:
        yaml.dump(legacy_content, f)

    script_path = os.path.join(PROJECT_DIR, 'migrate_tag_config.py')

    # 1. Run apply
    res = subprocess.run([
        sys.executable, script_path,
        '--input', str(legacy_file),
        '--apply'
    ], capture_output=True, text=True)
    assert res.returncode == 0
    assert "Migration successfully applied!" in res.stdout

    # Check that backup was created
    baks = list(tmp_path.glob("tags_config.yaml.bak.*"))
    assert len(baks) == 1

    # Check migrated content
    migrated = load_yaml(str(legacy_file))
    assert migrated['schema_version'] == 2
    assert migrated['anchor_tag_id'] is None

    # Load with TagRegistry to verify
    reg = TagRegistry(str(legacy_file))
    assert reg.revision == 1
    assert reg.anchor_tag_id is None
    assert reg.get_tag(17)['state'] == 'unconfirmed'

    # 2. Run rollback
    res_rb = subprocess.run([
        sys.executable, script_path,
        '--input', str(legacy_file),
        '--rollback'
    ], capture_output=True, text=True)
    assert res_rb.returncode == 0
    assert "Rollback completed successfully" in res_rb.stdout

    # Verify restored original content
    restored = load_yaml(str(legacy_file))
    assert 'schema_version' not in restored
    assert 'tag_17' in restored

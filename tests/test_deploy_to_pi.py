"""
test_deploy_to_pi.py - Unit tests for release packaging and manifest generation.
"""

import os
import sys
import pytest

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from deploy_to_pi import collect_deployment_files, generate_manifest, calc_sha256

def test_collect_deployment_files():
    files = collect_deployment_files(PROJECT_DIR)
    
    # Must include interface messages
    assert 'src/fake_tag_interfaces/msg/TagDetection.msg' in files
    assert 'src/fake_tag_interfaces/msg/TagDetectionArray.msg' in files
    assert 'src/fake_tag_interfaces/msg/TagMapUpdate.msg' in files
    assert 'src/fake_tag_interfaces/msg/TagMapAck.msg' in files
    assert 'src/fake_tag_interfaces/CMakeLists.txt' in files

    # Must include core node modules
    assert 'src/fake_tag_publisher/fake_tag_publisher/localization_node.py' in files
    assert 'src/fake_tag_publisher/fake_tag_publisher/video_tag_detector.py' in files
    assert 'src/fake_tag_publisher/fake_tag_publisher/multi_tag_fusion.py' in files
    assert 'src/fake_tag_publisher/fake_tag_publisher/tag_registry.py' in files
    assert 'src/fake_tag_publisher/fake_tag_publisher/geometry_transforms.py' in files

    # Must include root helpers
    assert 'start_termit.sh' in files
    assert 'migrate_tag_config.py' in files
    assert 'camera_extrinsics.yaml' in files

    # All paths must exist
    for rel_path, abs_path in files.items():
        assert os.path.exists(abs_path), f"File {rel_path} pointing to {abs_path} does not exist"

def test_generate_manifest():
    files = collect_deployment_files(PROJECT_DIR)
    manifest = generate_manifest(files, commit_hash="1c25988", release_tag="release-test")

    assert manifest["manifest_version"] == 1
    assert manifest["git_commit"] == "1c25988"
    assert manifest["release_tag"] == "release-test"
    assert manifest["file_count"] == len(files)
    assert len(manifest["files"]) == len(files)

    # Verify sha256 checksums
    for rel_path, meta in manifest["files"].items():
        assert len(meta["sha256"]) == 64
        assert meta["size_bytes"] > 0 or "resource" in rel_path or "__init__" in rel_path

import os
import sys
import glob
import time
import math
import shutil
import argparse
import yaml
from pathlib import Path

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.join(PROJECT_DIR, 'src', 'fake_tag_publisher', 'fake_tag_publisher')
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

try:
    from tag_registry import TagRegistry, ValidationError
except ImportError:
    TagRegistry = None

def load_yaml(filepath: str) -> dict:
    if not os.path.exists(filepath):
        return {}
    with open(filepath, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}

def save_yaml(filepath: str, data: dict):
    tmp_path = filepath + '.tmp'
    with open(tmp_path, 'w', encoding='utf-8') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp_path, filepath)

def migrate_dict(legacy_data: dict) -> dict:
    if legacy_data.get('schema_version') == 2 and 'tags' in legacy_data:
        data = dict(legacy_data)
        data['schema_version'] = 2
        return data

    default_size_mm = float(legacy_data.get('default_marker_size_mm', 100.0))
    ceiling_z = float(legacy_data.get('ceiling_z_m', 2.5))
    dict_name = str(legacy_data.get('dictionary', 'DICT_4X4_100'))

    v2_data = {
        'schema_version': 2,
        'config_epoch': int(legacy_data.get('config_epoch', 1)),
        'revision': int(legacy_data.get('revision', 1)),
        'dictionary': dict_name,
        'default_marker_size_mm': default_size_mm,
        'ceiling_z_m': ceiling_z,
        'anchor_tag_id': None,
        'tags': {}
    }

    raw_tags = {}
    if 'tags' in legacy_data and isinstance(legacy_data['tags'], dict):
        raw_tags = legacy_data['tags']
    else:
        for k, v in legacy_data.items():
            if k.startswith('tag_') or k.isdigit():
                raw_tags[k] = v

    for raw_id, raw_val in raw_tags.items():
        if not isinstance(raw_val, dict):
            continue
        norm_id = str(int(str(raw_id).replace('tag_', '')))
        
        pose = raw_val.get('pose', {})
        if not isinstance(pose, dict):
            pose = {}

        x = float(pose.get('x', raw_val.get('x', 0.0)))
        y = float(pose.get('y', raw_val.get('y', 0.0)))
        z = float(pose.get('z', raw_val.get('z', ceiling_z)))
        roll = float(pose.get('roll', raw_val.get('roll', math.pi)))
        pitch = float(pose.get('pitch', raw_val.get('pitch', 0.0)))
        yaw = float(pose.get('yaw', raw_val.get('yaw', 0.0)))

        if 'size_mm' in raw_val and raw_val['size_mm'] is not None:
            size_mm = float(raw_val['size_mm'])
        elif 'marker_size_m' in raw_val and raw_val['marker_size_m'] is not None:
            size_mm = float(raw_val['marker_size_m']) * 1000.0
        elif 'marker_size_mm' in raw_val and raw_val['marker_size_mm'] is not None:
            size_mm = float(raw_val['marker_size_mm'])
        else:
            size_mm = default_size_mm

        state = 'unconfirmed'
        enabled = bool(raw_val.get('enabled', True))

        v2_data['tags'][norm_id] = {
            'enabled': enabled,
            'state': state,
            'pose': {
                'x': x,
                'y': y,
                'z': z,
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw
            },
            'size_mm': size_mm,
            'source': 'legacy_migration',
            'observations': int(raw_val.get('observations', 0)),
            'parent_tags': list(raw_val.get('parent_tags', [])),
            'updated_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        }

    return v2_data

def find_latest_backup(target_file: str):
    pattern = target_file + '.bak.*'
    baks = sorted(glob.glob(pattern), reverse=True)
    return baks[0] if baks else None

def do_rollback(target_file: str) -> bool:
    latest = find_latest_backup(target_file)
    if not latest:
        print(f'No backup found matching {target_file}.bak.*')
        return False
    print(f'Restoring {target_file} from {latest} ...')
    shutil.copy2(latest, target_file)
    print('Rollback completed successfully.')
    return True

def main():
    parser = argparse.ArgumentParser(description='Migrate legacy tags_config.yaml to Schema v2.')
    parser.add_argument('--input', '-i', default='tags_config.yaml', help='Path to legacy tags_config.yaml')
    parser.add_argument('--output', '-o', default=None, help='Path for output Schema v2 yaml')
    parser.add_argument('--dry-run', action='store_true', help='Preview migration without saving')
    parser.add_argument('--apply', action='store_true', help='Apply migration and create backup')
    parser.add_argument('--rollback', action='store_true', help='Rollback to latest backup')
    args = parser.parse_args()

    input_path = os.path.abspath(args.input)
    output_path = os.path.abspath(args.output) if args.output else input_path

    if args.rollback:
        success = do_rollback(output_path)
        sys.exit(0 if success else 1)

    if not os.path.exists(input_path):
        print(f'Error: input file not found: {input_path}')
        sys.exit(1)

    legacy_data = load_yaml(input_path)
    migrated_data = migrate_dict(legacy_data)

    print("Migration summary:")
    print(f"   - Schema version: {migrated_data.get('schema_version')}")
    print(f"   - Tag count: {len(migrated_data.get('tags', {}))}")
    print(f"   - Anchor tag: {migrated_data.get('anchor_tag_id')} (unconfirmed)")

    if args.dry_run or (not args.apply):
        print('\n[DRY RUN] Generated Schema v2 YAML preview:')
        print(yaml.dump(migrated_data, default_flow_style=False, sort_keys=False))
        if not args.apply:
            print('Run with --apply to commit changes and create a backup.')
        return

    if os.path.exists(output_path):
        backup_path = f'{output_path}.bak.{int(time.time())}'
        print(f'Creating backup: {backup_path}')
        shutil.copy2(output_path, backup_path)

    print(f'Writing Schema v2 configuration to: {output_path}')
    save_yaml(output_path, migrated_data)

    if TagRegistry:
        try:
            reg = TagRegistry(output_path)
            print(f'Schema v2 verified successfully via TagRegistry (rev={reg.revision})')
        except Exception as e:
            print(f'Warning during registry verification: {e}')

    print('Migration successfully applied!')

if __name__ == '__main__':
    main()

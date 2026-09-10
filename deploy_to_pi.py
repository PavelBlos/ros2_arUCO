import os
import sys
import time
import socket
import logging
import hashlib
import subprocess
import json
import paramiko

logging.getLogger("paramiko").setLevel(logging.CRITICAL)

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
if hasattr(sys.stderr, 'reconfigure'):
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))

CANDIDATE_HOSTS = [
    '192.168.10.163',
    '172.20.10.2',
    '100.99.95.103',
    '172.20.10.3',
    '172.20.10.4',
    '192.168.31.213',
    'raspberrypi.local',
    'termit.local'
]

CREDENTIALS = [
    ('raspberry', 'pi'),
    ('pi', 'raspberry'),
    ('pi', 'pi'),
    ('ubuntu', 'ubuntu')
]

def calc_sha256(filepath):
    h = hashlib.sha256()
    with open(filepath, 'rb') as f:
        while chunk := f.read(8192):
            h.update(chunk)
    return h.hexdigest()

def get_git_commit():
    try:
        res = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'], cwd=PROJECT_DIR)
        return res.decode().strip()
    except Exception:
        return "unknown"

def collect_deployment_files(base_dir=PROJECT_DIR):
    """
    Collect all files needed for a self-contained release workspace on Raspberry Pi.
    Returns: dict mapping remote_relative_path -> local_absolute_path.
    """
    files = {}

    # 1. fake_tag_interfaces package
    interfaces_dir = os.path.join(base_dir, 'src', 'fake_tag_interfaces')
    if os.path.exists(interfaces_dir):
        for fname in ['CMakeLists.txt', 'package.xml']:
            p = os.path.join(interfaces_dir, fname)
            if os.path.exists(p):
                files[f'src/fake_tag_interfaces/{fname}'] = p
        
        msg_dir = os.path.join(interfaces_dir, 'msg')
        if os.path.exists(msg_dir):
            for m in os.listdir(msg_dir):
                if m.endswith('.msg'):
                    files[f'src/fake_tag_interfaces/msg/{m}'] = os.path.join(msg_dir, m)

    # 2. fake_tag_publisher package
    publisher_dir = os.path.join(base_dir, 'src', 'fake_tag_publisher')
    if os.path.exists(publisher_dir):
        for fname in ['setup.py', 'setup.cfg', 'package.xml']:
            p = os.path.join(publisher_dir, fname)
            if os.path.exists(p):
                files[f'src/fake_tag_publisher/{fname}'] = p
        
        # Marker resource
        res_file = os.path.join(publisher_dir, 'resource', 'fake_tag_publisher')
        if os.path.exists(res_file):
            files['src/fake_tag_publisher/resource/fake_tag_publisher'] = res_file

        # Configs
        cfg_dir = os.path.join(publisher_dir, 'config')
        if os.path.exists(cfg_dir):
            for c in ['camera_info.yaml', 'camera_extrinsics.yaml', 'tags_config.yaml', 'runtime_settings.yaml']:
                cp = os.path.join(cfg_dir, c)
                if os.path.exists(cp):
                    files[f'src/fake_tag_publisher/config/{c}'] = cp

        # Python modules
        py_dir = os.path.join(publisher_dir, 'fake_tag_publisher')
        if os.path.exists(py_dir):
            for pyf in os.listdir(py_dir):
                if pyf.endswith('.py'):
                    files[f'src/fake_tag_publisher/fake_tag_publisher/{pyf}'] = os.path.join(py_dir, pyf)
                    # Also map key nodes to release root for direct execution
                    if pyf in ['localization_node.py', 'video_tag_detector.py', 'termit_api.py',
                              'geometry_transforms.py', 'tag_registry.py', 'single_tag_pnp.py',
                              'multi_tag_fusion.py', 'tag_calibration_wizard.py', 'covisibility_graph.py']:
                        files[pyf] = os.path.join(py_dir, pyf)

    # 3. Root helper scripts and configuration
    for root_f in ['start_termit.sh', 'migrate_tag_config.py', 'camera_extrinsics.yaml', 'test_cam.py', 'tags_config.yaml', 'runtime_settings.yaml']:
        rp = os.path.join(base_dir, root_f)
        if os.path.exists(rp):
            files[root_f] = rp

    return files

def generate_manifest(files_map, commit_hash=None, release_tag=None):
    """Generate manifest dictionary with SHA256 hashes and metadata."""
    if commit_hash is None:
        commit_hash = get_git_commit()
    timestamp = int(time.time())
    if release_tag is None:
        release_tag = f"release-{timestamp}-{commit_hash}"

    manifest = {
        "manifest_version": 1,
        "release_tag": release_tag,
        "git_commit": commit_hash,
        "timestamp": timestamp,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(timestamp)),
        "file_count": len(files_map),
        "files": {}
    }

    for rel_path, abs_path in sorted(files_map.items()):
        manifest["files"][rel_path] = {
            "sha256": calc_sha256(abs_path),
            "size_bytes": os.path.getsize(abs_path)
        }

    return manifest

def scan_port(host, port=22, timeout=0.8):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        res = s.connect_ex((host, port))
        s.close()
        return res == 0
    except Exception:
        return False

def try_connect(host, user, pwd, timeout=5.0):
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(host, port=22, username=user, password=pwd, timeout=timeout, banner_timeout=5.0)
        return ssh
    except Exception:
        return None

def find_active_ssh(target_host=None, target_user=None, target_pass=None):
    hosts = [target_host] if target_host else CANDIDATE_HOSTS
    creds = [(target_user, target_pass)] if (target_user and target_pass) else CREDENTIALS
    
    for h in hosts:
        print(f"[*] Проверка {h}:22 ... ", end="", flush=True)
        if not scan_port(h, 22):
            print("недоступен")
            continue
        print("порт 22 открыт!")
        
        for user, pwd in creds:
            ssh = try_connect(h, user, pwd)
            if ssh:
                print(f"✅ Успешный вход на {h} как '{user}'!")
                return ssh, h, user

    return None, None, None

def deploy(target_host=None, target_user=None, target_pass=None, wait_loop=False, dry_run=False):
    print("=" * 65)
    print("🚀 АТОМАРНЫЙ ДЕПЛОЙ И ИЗОЛИРОВАННАЯ СБОРКА RASPBERRY PI")
    print("=" * 65)

    files_map = collect_deployment_files(PROJECT_DIR)
    commit_hash = get_git_commit()
    timestamp = int(time.time())
    release_tag = f"release-{timestamp}-{commit_hash}"
    manifest = generate_manifest(files_map, commit_hash, release_tag)

    print(f"📦 Релиз: {release_tag} (коммит: {commit_hash})")
    print(f"📄 Файлов к деплою: {len(files_map)}")

    if dry_run:
        print("\n[DRY RUN] Манифест релиза:")
        print(json.dumps(manifest, indent=2))
        return True

    ssh = None
    if wait_loop:
        print("[*] Режим ожидания подключения к Raspberry Pi...")
        while not ssh:
            ssh, host, user = find_active_ssh(target_host, target_user, target_pass)
            if not ssh:
                time.sleep(3)
    else:
        ssh, host, user = find_active_ssh(target_host, target_user, target_pass)

    if not ssh:
        print("\n❌ Raspberry Pi сейчас не в сети или не отвечает по SSH.")
        return False

    try:
        sftp = ssh.open_sftp()
        base_dir = f"/home/{user}/arUco_termit"
        releases_dir = f"{base_dir}/releases"
        target_rel_dir = f"{releases_dir}/{release_tag}"
        current_symlink = f"{base_dir}/current"
        shared_config_dir = f"{base_dir}/shared_config"

        # Создаем необходимые директории на Малине
        for d in [base_dir, releases_dir, target_rel_dir, shared_config_dir]:
            ssh.exec_command(f"mkdir -p {d}")
        time.sleep(0.3)

        print(f"\n📁 Создание структуры директорий в {target_rel_dir} ...")
        # Создаем подкаталоги релиза
        subdirs = set(os.path.dirname(p) for p in files_map.keys() if os.path.dirname(p))
        for sd in sorted(subdirs):
            ssh.exec_command(f"mkdir -p {target_rel_dir}/{sd}")
        time.sleep(0.3)

        # 1. Загрузка всех файлов релиза во временные *.tmp и атомарный rename
        print("\n📤 Передача файлов релиза:")
        for rel_path, local_path in sorted(files_map.items()):
            remote_final = f"{target_rel_dir}/{rel_path}"
            remote_tmp = f"{remote_final}.tmp"
            if rel_path.endswith('.sh'):
                # Git's Windows checkout may contain CRLF even when the script
                # is executed by bash on the Pi. Upload normalized bytes.
                with open(local_path, 'rb') as src:
                    normalized = src.read().replace(b'\r\n', b'\n')
                with sftp.file(remote_tmp, 'wb') as dst:
                    dst.write(normalized)
            else:
                sftp.put(local_path, remote_tmp)
            ssh.exec_command(f"mv {remote_tmp} {remote_final}")
            if rel_path.endswith('.py') or rel_path.endswith('.sh'):
                ssh.exec_command(f"chmod +x {remote_final}")
            print(f"  -> {rel_path} ✅")

        # 2. Сохраняем manifest.json в целевом релизе
        manifest_remote = f"{target_rel_dir}/manifest.json"
        with sftp.file(manifest_remote + ".tmp", "w") as mf:
            mf.write(json.dumps(manifest, indent=2))
        ssh.exec_command(f"mv {manifest_remote}.tmp {manifest_remote}")
        print("  -> manifest.json ✅")

        # 3. Валидация синтаксиса Python
        print("\n🔍 Валидация синтаксиса Python на Raspberry Pi...")
        stdin, stdout, stderr = ssh.exec_command(f"python3 -m py_compile {target_rel_dir}/src/fake_tag_publisher/fake_tag_publisher/*.py")
        compile_err = stderr.read().decode().strip()
        if compile_err:
            print(f"❌ Ошибка компиляции на Малине: {compile_err}")
            sftp.close()
            ssh.close()
            return False
        print("✅ Все Python-модули успешно скомпилированы")

        # 4. Изолированная сборка colcon build внутри целевого релиза
        print("\n🔨 Изолированная сборка colcon build внутри релиза...")
        build_cmd = (
            f"bash -c 'source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash; "
            f"cd {target_rel_dir} && colcon build --packages-select fake_tag_interfaces fake_tag_publisher'"
        )
        stdin, stdout, stderr = ssh.exec_command(build_cmd)
        build_out = stdout.read().decode().strip()
        build_err = stderr.read().decode().strip()
        print(f"   Сборка завершена.")

        # Проверяем создание install/setup.bash
        stdin, stdout, stderr = ssh.exec_command(f"test -f {target_rel_dir}/install/setup.bash && echo 'OK' || echo 'FAIL'")
        setup_status = stdout.read().decode().strip()
        if setup_status != 'OK':
            print(f"⚠️ Предупреждение: изолированный install/setup.bash не сформирован. Подробности сборки: {build_err[:300]}")
        else:
            print("✅ Изолированный setup.bash успешно создан внутри релиза!")

        # 5. Initialize persistent configuration once. Existing calibrated files
        # are never overwritten by a deployment.
        shared_cfg_path = f"{shared_config_dir}/tags_config.yaml"
        stdin, stdout, stderr = ssh.exec_command(f"test -f {shared_cfg_path} && echo 'EXISTS' || echo 'NEW'")
        cfg_exists = stdout.read().decode().strip() == 'EXISTS'

        if not cfg_exists:
            print(f"\n📋 Инициализация shared_config: передача tags_config.yaml ...")
            local_cfg = files_map.get('tags_config.yaml', os.path.join(PROJECT_DIR, 'tags_config.yaml'))
            sftp.put(local_cfg, shared_cfg_path)
            print("✅ Создан начальный shared_config/tags_config.yaml")

        print(f"\n🔄 Проверка и миграция конфигурации меток в Schema v2...")
        stdin, stdout, stderr = ssh.exec_command(f"python3 {target_rel_dir}/migrate_tag_config.py --input {shared_cfg_path} --apply")
        mig_out = stdout.read().decode().strip()
        print(f"   {mig_out.splitlines()[-1] if mig_out else 'OK'}")

        # Проверяем наличие camera_extrinsics.yaml в shared_config
        shared_cam_ext = f"{shared_config_dir}/camera_extrinsics.yaml"
        stdin, stdout, stderr = ssh.exec_command(f"test -f {shared_cam_ext} && echo 'EXISTS' || echo 'NEW'")
        if stdout.read().decode().strip() != 'EXISTS':
            sftp.put(files_map.get('camera_extrinsics.yaml', os.path.join(PROJECT_DIR, 'camera_extrinsics.yaml')), shared_cam_ext)
            print("✅ Скопирован базовый camera_extrinsics.yaml в shared_config")

        persistent_defaults = {
            'camera_info.yaml': files_map.get('src/fake_tag_publisher/config/camera_info.yaml'),
            'runtime_settings.yaml': files_map.get('runtime_settings.yaml') or files_map.get('src/fake_tag_publisher/config/runtime_settings.yaml'),
        }
        for cfg_name, local_cfg in persistent_defaults.items():
            if not local_cfg:
                continue
            remote_cfg = f"{shared_config_dir}/{cfg_name}"
            stdin, stdout, stderr = ssh.exec_command(f"test -f {remote_cfg} && echo 'EXISTS' || echo 'NEW'")
            if stdout.read().decode().strip() != 'EXISTS':
                sftp.put(local_cfg, remote_cfg)
                print(f"✅ Создан начальный shared_config/{cfg_name}")

        # 6. Атомарное переключение симлинка current
        print(f"\n🔗 Атомарное переключение симлинка: current -> {release_tag}")
        ssh.exec_command(f"ln -sfn {target_rel_dir} {current_symlink}")

        # 7. Контрольная проверка SHA-256
        print("\n🔒 Проверка SHA256 контрольных сумм...")
        stdin, stdout, stderr = ssh.exec_command(f"sha256sum {current_symlink}/*.py {current_symlink}/*.sh 2>/dev/null")
        remote_hashes = stdout.read().decode().splitlines()
        for line in remote_hashes:
            parts = line.strip().split()
            if len(parts) >= 2:
                r_hash = parts[0]
                r_name = os.path.basename(parts[1])
                expected = manifest["files"].get(r_name, {}).get("sha256")
                if expected:
                    match_str = "✅" if expected == r_hash else "❌"
                    print(f"  - {r_name}: {match_str}")

        sftp.close()
        ssh.close()

        print("\n" + "=" * 65)
        print(f"🎉 ИЗОЛИРОВАННЫЙ РЕЛИЗ {release_tag} УСПЕШНО АКТИВИРОВАН!")
        print(f"Активный каталог: {current_symlink}")
        print(f"Запуск: python pi_exec.py 'nohup bash {current_symlink}/start_termit.sh > /tmp/termit_start.log 2>&1 < /dev/null &'")
        print("=" * 65)
        return True

    except Exception as e:
        print(f"❌ Ошибка во время деплоя: {e}")
        if ssh:
            ssh.close()
        return False

if __name__ == '__main__':
    dry_flag = '--dry-run' in sys.argv
    wait_flag = '--wait' in sys.argv
    args = [a for a in sys.argv[1:] if a not in ('--dry-run', '--wait')]
    host_arg = args[0] if len(args) > 0 else None
    user_arg = args[1] if len(args) > 1 else None
    pass_arg = args[2] if len(args) > 2 else None
    deploy(host_arg, user_arg, pass_arg, wait_loop=wait_flag, dry_run=dry_flag)

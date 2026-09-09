import os
import sys
import time
import socket
import logging
import hashlib
import subprocess
import paramiko

logging.getLogger("paramiko").setLevel(logging.CRITICAL)

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
if hasattr(sys.stderr, 'reconfigure'):
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))

REQUIRED_FILES = {
    'localization_node.py': os.path.join(PROJECT_DIR, 'src', 'fake_tag_publisher', 'fake_tag_publisher', 'localization_node.py'),
    'video_tag_detector.py': os.path.join(PROJECT_DIR, 'src', 'fake_tag_publisher', 'fake_tag_publisher', 'video_tag_detector.py'),
    'termit_api.py': os.path.join(PROJECT_DIR, 'src', 'fake_tag_publisher', 'fake_tag_publisher', 'termit_api.py'),
    'start_termit.sh': os.path.join(PROJECT_DIR, 'start_termit.sh'),
    'tags_config.yaml': os.path.join(PROJECT_DIR, 'tags_config.yaml'),
    'test_cam.py': os.path.join(PROJECT_DIR, 'test_cam.py')
}

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

def get_git_commit():
    try:
        res = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'], cwd=PROJECT_DIR)
        return res.decode().strip()
    except Exception:
        return "unknown"

def calc_sha256(filepath):
    h = hashlib.sha256()
    with open(filepath, 'rb') as f:
        while chunk := f.read(8192):
            h.update(chunk)
    return h.hexdigest()

def deploy(target_host=None, target_user=None, target_pass=None, wait_loop=False):
    print("=" * 65)
    print("🚀 АТОМАРНЫЙ ДЕПЛОЙ И СИНХРОНИЗАЦИЯ С RASPBERRY PI")
    print("=" * 65)
    
    # 1. Проверка наличия всех обязательных локальных файлов
    missing = []
    local_hashes = {}
    for name, path in REQUIRED_FILES.items():
        if not os.path.exists(path):
            missing.append(f"{name} ({path})")
        else:
            local_hashes[name] = calc_sha256(path)
            
    if missing:
        print("\n❌ КРИТИЧЕСКАЯ ОШИБКА: отсутствуют обязательные файлы:")
        for m in missing:
            print(f"  - {m}")
        sys.exit(1)

    print("✅ Все локальные файлы проверены (sha256 рассчитаны)")

    ssh, host, user = find_active_ssh(target_host, target_user, target_pass)
    if not ssh:
        print("\n❌ Малина сейчас не в сети или не отвечает по SSH.")
        return False

    try:
        sftp = ssh.open_sftp()
        commit_hash = get_git_commit()
        timestamp = int(time.time())
        release_tag = f"release-{timestamp}-{commit_hash}"
        base_dir = f"/home/{user}/arUco_termit"
        releases_dir = f"{base_dir}/releases"
        target_rel_dir = f"{releases_dir}/{release_tag}"
        current_symlink = f"{base_dir}/current"
        shared_config_dir = f"{base_dir}/shared_config"

        # Создаем необходимые директории на Малине
        for d in [base_dir, releases_dir, target_rel_dir, shared_config_dir]:
            ssh.exec_command(f"mkdir -p {d}")
        time.sleep(0.3)

        print(f"\n📦 Подготовка нового релиза: {release_tag}")
        
        # 2. Загрузка файлов во временные имена (*.tmp) внутри релиза
        for name, local_path in REQUIRED_FILES.items():
            tmp_remote = f"{target_rel_dir}/{name}.tmp"
            final_remote = f"{target_rel_dir}/{name}"
            print(f"  -> Передача {name} ... ", end="", flush=True)
            sftp.put(local_path, tmp_remote)
            # Переименование в релизной директории
            ssh.exec_command(f"mv {tmp_remote} {final_remote}")
            if name.endswith('.py') or name.endswith('.sh'):
                ssh.exec_command(f"chmod +x {final_remote}")
            print("OK")

        # 3. Синтаксическая проверка py_compile на Raspberry Pi
        print("\n🔍 Валидация синтаксиса Python на Raspberry Pi...")
        stdin, stdout, stderr = ssh.exec_command(f"python3 -m py_compile {target_rel_dir}/*.py")
        compile_err = stderr.read().decode().strip()
        if compile_err:
            print(f"❌ Ошибка компиляции на Малине: {compile_err}")
            print("Откат: релиз не активирован!")
            sftp.close()
            ssh.close()
            return False
        print("✅ Все Python-модули успешно скомпилированы без ошибок")

        # 4. Атомарное переключение симлинка current
        print(f"\n🔗 Атомарное переключение симлинка: current -> {release_tag}")
        ssh.exec_command(f"ln -sfn {target_rel_dir} {current_symlink}")

        # 5. Синхронизация постоянного каталога arUco_termit и ROS 2 share
        print("\n🔄 Синхронизация с постоянными путями ROS 2...")
        # Копируем в корень arUco_termit
        for name, local_path in REQUIRED_FILES.items():
            sftp.put(local_path, f"{base_dir}/{name}")
            if name.endswith('.py') or name.endswith('.sh'):
                ssh.exec_command(f"chmod +x {base_dir}/{name}")

        # Копируем в ros2_ws/src
        ros2_pkg_dir = f"/home/{user}/ros2_ws/src/fake_tag_publisher/fake_tag_publisher"
        ssh.exec_command(f"mkdir -p {ros2_pkg_dir}")
        for name in ['localization_node.py', 'video_tag_detector.py', 'termit_api.py']:
            local_path = REQUIRED_FILES[name]
            sftp.put(local_path, f"{ros2_pkg_dir}/{name}")
            ssh.exec_command(f"chmod +x {ros2_pkg_dir}/{name}")

        # Копируем tags_config.yaml в share
        ros2_share_cfg = f"/home/{user}/ros2_ws/install/fake_tag_publisher/share/fake_tag_publisher/config"
        ssh.exec_command(f"mkdir -p {ros2_share_cfg}")
        sftp.put(REQUIRED_FILES['tags_config.yaml'], f"{ros2_share_cfg}/tags_config.yaml")
        sftp.put(REQUIRED_FILES['tags_config.yaml'], f"{shared_config_dir}/tags_config.yaml")

        # 6. Проверка контрольных сумм sha256 на Малине
        print("\n🔒 Проверка SHA256 контрольных сумм...")
        stdin, stdout, stderr = ssh.exec_command(f"sha256sum {current_symlink}/*")
        remote_hashes = stdout.read().decode().splitlines()
        all_match = True
        for line in remote_hashes:
            parts = line.strip().split()
            if len(parts) >= 2:
                r_hash = parts[0]
                r_name = os.path.basename(parts[1])
                if r_name in local_hashes:
                    if local_hashes[r_name] == r_hash:
                        print(f"  - {r_name}: SHA256 совпадает ✅")
                    else:
                        print(f"  - {r_name}: ХЭШ НЕ СОВПАДАЕТ ❌ ({local_hashes[r_name][:8]} vs {r_hash[:8]})")
                        all_match = False

        sftp.close()
        ssh.close()

        if all_match:
            print("\n" + "=" * 65)
            print(f"🎉 РЕЛИЗ {release_tag} УСПЕШНО АКТИВИРОВАН!")
            print(f"Активный путь: {current_symlink}")
            print(f"Запуск: python pi_exec.py 'nohup bash {current_symlink}/start_termit.sh > /tmp/termit_start.log 2>&1 < /dev/null &'")
            print("=" * 65)
            return True
        else:
            print("\n⚠️ Предупреждение: не все хэши совпали.")
            return False

    except Exception as e:
        print(f"❌ Ошибка во время передачи: {e}")
        if ssh:
            ssh.close()
        return False

if __name__ == '__main__':
    wait_flag = '--wait' in sys.argv
    args = [a for a in sys.argv[1:] if a != '--wait']
    host_arg = args[0] if len(args) > 0 else None
    user_arg = args[1] if len(args) > 1 else None
    pass_arg = args[2] if len(args) > 2 else None
    deploy(host_arg, user_arg, pass_arg, wait_loop=wait_flag)


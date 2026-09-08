#!/usr/bin/env python3
"""
Deploy Termit Omni Robot Files to Raspberry Pi
===============================================
Скрипт синхронизации файлов проекта с Raspberry Pi по SSH/SFTP.
"""

import os
import sys
import time
import socket
import logging
import paramiko

logging.getLogger("paramiko").setLevel(logging.CRITICAL)

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
if hasattr(sys.stderr, 'reconfigure'):
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))

FILES_TO_DEPLOY = [
    'localization_node_remote.py',
    'termit_api.py',
    'termit_ros2_node.py',
    'video_tag_detector_remote.py',
    'example_api_usage.py',
    'web_pult.py',
    'start_termit.sh',
    'test_cam.py'
]

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

def try_connect(host, user, pwd, timeout=10.0):
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(host, port=22, username=user, password=pwd, timeout=timeout, banner_timeout=15.0, auth_timeout=15.0)
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

def deploy(target_host=None, target_user=None, target_pass=None, wait_loop=False):
    print("=" * 65)
    print("🚀 СИНХРОНИЗАЦИЯ ФАЙЛОВ С RASPBERRY PI")
    print("=" * 65)
    
    ssh = None
    host = None
    user = None

    if wait_loop:
        print("⏳ Ожидание появления Raspberry Pi в сети (Ctrl+C для отмены)...")
        while not ssh:
            ssh, host, user = find_active_ssh(target_host, target_user, target_pass)
            if not ssh:
                time.sleep(2.0)
    else:
        ssh, host, user = find_active_ssh(target_host, target_user, target_pass)

    if not ssh:
        print("\n❌ Малина сейчас не в сети или не отвечает по SSH.")
        print("\n💡 Как передать файлы:")
        print("1. Включите питание Raspberry Pi и подключите её к Wi-Fi / Точке доступа / Tailscale.")
        print("2. Если известен IP-адрес, запустите:")
        print("     python deploy_to_pi.py <IP_МАЛИНЫ> [USER] [PASSWORD]")
        print("   Пример: python deploy_to_pi.py 192.168.1.100 raspberry pi")
        print("3. Или запустите режим ожидания подключения:")
        print("     python deploy_to_pi.py --wait")
        return False

    try:
        sftp = ssh.open_sftp()
        remote_dir = f"/home/{user}/arUco_termit"

        # Создание каталога при необходимости
        try:
            sftp.stat(remote_dir)
        except FileNotFoundError:
            print(f"📁 Создание каталога на Малине: {remote_dir}")
            ssh.exec_command(f"mkdir -p {remote_dir}")
            time.sleep(0.5)

        print(f"\n📤 Передача файлов в {remote_dir} на {host}...")
        for filename in FILES_TO_DEPLOY:
            local_path = os.path.join(PROJECT_DIR, filename)
            if not os.path.exists(local_path):
                continue
            
            remote_path = f"{remote_dir}/{filename}"
            print(f"  -> Отправка {filename} ... ", end="", flush=True)
            sftp.put(local_path, remote_path)
            if filename.endswith('.py'):
                ssh.exec_command(f"chmod +x {remote_path}")
            print("✅ OK")

        # Проверяем наличие пакета ROS 2 fake_tag_publisher
        stdin, stdout, stderr = ssh.exec_command("find /home/raspberry/ros2_ws/src -type d -name fake_tag_publisher 2>/dev/null")
        ros_dirs = [d.strip() for d in stdout.read().decode().splitlines() if d.strip()]
        
        target_pkg_dir = "/home/raspberry/ros2_ws/src/fake_tag_publisher/fake_tag_publisher"
        print(f"\n📦 Обновляем файлы в пакете ROS 2 ({target_pkg_dir}):")
        
        # localization_node.py
        local_loc = os.path.join(PROJECT_DIR, 'localization_node_remote.py')
        if os.path.exists(local_loc):
            sftp.put(local_loc, f"{target_pkg_dir}/localization_node.py")
            ssh.exec_command(f"chmod +x {target_pkg_dir}/localization_node.py")
            print(f"  -> localization_node.py обновлен в ROS 2 ✅")

        # video_tag_detector.py
        local_vid = os.path.join(PROJECT_DIR, 'video_tag_detector_remote.py')
        if os.path.exists(local_vid):
            sftp.put(local_vid, f"{target_pkg_dir}/video_tag_detector.py")
            ssh.exec_command(f"chmod +x {target_pkg_dir}/video_tag_detector.py")
            print(f"  -> video_tag_detector.py обновлен в ROS 2 ✅")
            
        # termit_api.py & termit_ros2_node.py
        for fn in ['termit_api.py', 'termit_ros2_node.py', 'example_api_usage.py']:
            lp = os.path.join(PROJECT_DIR, fn)
            if os.path.exists(lp):
                sftp.put(lp, f"{target_pkg_dir}/{fn}")
                ssh.exec_command(f"chmod +x {target_pkg_dir}/{fn}")
                print(f"  -> {fn} обновлен в ROS 2 ✅")

        # tags_config.yaml
        local_tags = os.path.join(PROJECT_DIR, 'tags_config.yaml')
        if os.path.exists(local_tags):
            for t_dir in [
                "/home/raspberry/ros2_ws/src/fake_tag_publisher/config",
                "/home/raspberry/ros2_ws/install/fake_tag_publisher/share/fake_tag_publisher/config"
            ]:
                try:
                    sftp.put(local_tags, f"{t_dir}/tags_config.yaml")
                    print(f"  -> tags_config.yaml обновлен в {t_dir} ✅")
                except Exception as e:
                    print(f"  -> Ошибка обновления tags_config.yaml в {t_dir}: {e}")

        sftp.close()
        ssh.close()
        
        print("\n" + "=" * 65)
        print(f"🎉 ВСЕ ФАЙЛЫ УСПЕШНО ЗАГРУЖЕНЫ НА RASPBERRY PI ({host})!")
        print(f"Для запуска выполните по SSH:")
        print(f"  python3 {remote_dir}/localization_node_remote.py")
        print("=" * 65)
        return True

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

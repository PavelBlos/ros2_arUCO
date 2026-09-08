import sys
import logging
import paramiko
import socket

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
if hasattr(sys.stderr, 'reconfigure'):
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

logging.getLogger("paramiko").setLevel(logging.CRITICAL)

HOSTS = ['192.168.10.163', '172.20.10.2', '100.99.95.103', '172.20.10.3', '172.20.10.4', '192.168.31.213', 'raspberrypi.local']
CREDS = [('raspberry', 'pi'), ('pi', 'raspberry'), ('pi', 'pi')]

def is_port_open(host, port=22, timeout=0.8):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        res = s.connect_ex((host, port))
        s.close()
        return res == 0
    except Exception:
        return False

def run(cmd, target_ip=None):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    hosts = [target_ip] if target_ip else HOSTS
    connected = False
    active_host = None
    
    for h in hosts:
        if not is_port_open(h, 22):
            continue
        for user, pwd in CREDS:
            try:
                ssh.connect(h, username=user, password=pwd, timeout=2.0, banner_timeout=2.0)
                connected = True
                active_host = h
                break
            except Exception:
                continue
        if connected:
            break
            
    if not connected:
        print(f"[-] Не удалось подключиться к Raspberry Pi по SSH ({hosts})", file=sys.stderr)
        return

    full_cmd = f"echo pi | sudo -S {cmd}" if cmd.startswith('sudo ') else cmd
    stdin, stdout, stderr = ssh.exec_command(full_cmd)
    out = stdout.read().decode('utf-8', errors='ignore')
    err = stderr.read().decode('utf-8', errors='ignore')
    ssh.close()
    
    if out:
        print(out, end='')
    if err:
        clean_err = '\n'.join([line for line in err.split('\n') if '[sudo] password for' not in line])
        if clean_err.strip():
            print(clean_err, file=sys.stderr)

if __name__ == '__main__':
    if len(sys.argv) > 1:
        run(' '.join(sys.argv[1:]))

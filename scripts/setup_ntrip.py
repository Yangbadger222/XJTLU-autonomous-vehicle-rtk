import sys
import socket
import base64
import yaml
import os
import json
import getpass
import argparse

# --- File Paths ---
CONFIG_CACHE = os.path.expanduser("~/.ntrip_config.json")
CRED_CACHE = os.path.expanduser("~/.ntrip_credentials.json")
OUT_PARAMS_FILE = "/tmp/um982_cors.yaml"
ENV_FILE = "/tmp/ntrip_env.sh"

def test_ntrip(host, port, mountpoint, username, password, timeout_sec=5.0):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout_sec)
        s.connect((host, port))
        
        req = f"GET /{mountpoint} HTTP/1.0\r\n"
        req += "User-Agent: NTRIP Client/1.0\r\n"
        auth = base64.b64encode(f"{username}:{password}".encode()).decode()
        req += f"Authorization: Basic {auth}\r\n"
        req += "\r\n"
        
        s.sendall(req.encode())
        resp = s.recv(1024).decode(errors='ignore')
        s.close()
        
        if "ICY 200 OK" in resp or "HTTP/1.1 200 OK" in resp:
            return True, "Success"
        elif "401 Unauthorized" in resp:
            return False, "Unauthorized (check username/password)"
        elif "404 Not Found" in resp:
            return False, "Mountpoint not found"
        else:
            first_line = resp.splitlines()[0] if resp else 'No response'
            return False, f"Unexpected response: {first_line}"
    except Exception as e:
        return False, str(e)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--setup', action='store_true', help='Interactively configure server')
    parser.add_argument('--logout', action='store_true', help='Clear saved credentials')
    parser.add_argument('--status', action='store_true', help='Check current login status')
    args = parser.parse_args()

    # ==========================================
    # LOGOUT MODE
    # ==========================================
    if args.logout:
        if os.path.exists(CRED_CACHE):
            os.remove(CRED_CACHE)
            print("Logged out. Cached credentials removed (Server config retained).")
        else:
            print("No cached credentials found.")
        sys.exit(0)

    # ==========================================
    # STATUS CHECK MODE
    # ==========================================
    if args.status:
        if not os.path.exists(CRED_CACHE) or not os.path.exists(CONFIG_CACHE):
            print("Status: ⚪ Not configured or logged in.")
            sys.exit(0)
            
        try:
            with open(CONFIG_CACHE, 'r') as f:
                cfg = json.load(f)
            with open(CRED_CACHE, 'r') as f:
                creds = json.load(f)
                username, password = creds.get('username'), creds.get('password')
        except json.JSONDecodeError:
            print("Status: ❌ Error reading cache. Run 'make ntrip-setup'.")
            sys.exit(1)
            
        print(f"Current User: {username}")
        print(f"Server: {cfg['host']}:{cfg['port']} (Mountpoint: {cfg['mountpoint']})")
        
        success, msg = test_ntrip(cfg['host'], cfg['port'], cfg['mountpoint'], username, password, timeout_sec=1.0)
        if success:
            print("Status: ✅ ACTIVE (Connection successful)")
        else:
            print(f"Status: ❌ EXPIRED / FAILED ({msg})")
            print("Hint: Run 'make ntrip-login' or 'make ntrip-setup' to update.")
        sys.exit(0)

    # ==========================================
    # SETUP MODE (Interactive Prompts)
    # ==========================================
    host, port, mountpoint = "", 8002, ""
    username, password = "", ""

    if args.setup:
        print("\n================================================")
        print("          NTRIP SERVER CONFIGURATION")
        print("================================================")
        
        host = input("1. Host IP (e.g., 140.143.212.42): ").strip()
        
        port_in = input("2. Port [Press Enter for 8002]: ").strip()
        port = int(port_in) if port_in else 8002
        
        mountpoint = input("3. Mountpoint (e.g., RTCM32_GRECJ2): ").strip()
        
        username = input("4. Username: ").strip()
        
        # Using standard input instead of getpass so you can verify you pasted correctly
        password = input("5. Password: ").strip()

        # Save the server configuration
        with open(CONFIG_CACHE, 'w') as f:
            json.dump({'host': host, 'port': port, 'mountpoint': mountpoint}, f)

    # ==========================================
    # LOGIN MODE (Normal flow)
    # ==========================================
    else:
        if not os.path.exists(CONFIG_CACHE):
            print("❌ No server configuration found.")
            print("Please run 'make ntrip-setup' first to configure the IP and Mountpoint.")
            sys.exit(1)
            
        with open(CONFIG_CACHE, 'r') as f:
            cfg = json.load(f)
            host, port, mountpoint = cfg['host'], cfg['port'], cfg['mountpoint']

    # Handle Credentials Logging (Only if we didn't just ask for them in setup)
    if not args.setup:
        if os.path.exists(CRED_CACHE):
            try:
                with open(CRED_CACHE, 'r') as f:
                    creds = json.load(f)
                    c_user, c_pass = creds.get('username'), creds.get('password')
                    
                if c_user and c_pass:
                    print(f"Found previous credentials for user: {c_user}")
                    choice = input("Do you want to restore this session? [Y/n]: ").strip().lower()
                    if choice in ['', 'y', 'yes']:
                        username, password = c_user, c_pass
                    else:
                        print("\nProceeding with new login...")
            except json.JSONDecodeError:
                pass

    # Fallback if username/password aren't populated yet
    if not username or not password:
        print(f"\nConnecting to {host}:{port} (Mountpoint: {mountpoint})")
        username = input("Username: ").strip()
        password = getpass.getpass("Password: ").strip()

    # ==========================================
    # CONNECTION TEST & YAML GENERATION
    # ==========================================
    print("\nTesting connection...")
    success, msg = test_ntrip(host, port, mountpoint, username, password, timeout_sec=5.0)
    
    if success:
        print("✅ Connection test successful!")
        # Save working credentials
        with open(CRED_CACHE, 'w') as f:
            json.dump({'username': username, 'password': password}, f)
    else:
        print(f"❌ Connection test failed: {msg}")
        if os.path.exists(CRED_CACHE):
            os.remove(CRED_CACHE)
        sys.exit(1)

    # Generate ROS YAML
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_dir = os.path.dirname(script_dir)
    params_file = os.path.join(repo_dir, "src", "bringup", "config", "master_params.yaml")
    
    if not os.path.exists(params_file):
        print(f"Error: master_params.yaml not found at {params_file}")
        sys.exit(1)
        
    with open(params_file, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f)
        
    um982_config = params.get('/um982_rtk_driver', {})
    if 'ros__parameters' not in um982_config:
        um982_config['ros__parameters'] = {}
    if 'ntrip' not in um982_config['ros__parameters']:
        um982_config['ros__parameters']['ntrip'] = {}
        
    ntrip_cfg = um982_config['ros__parameters']['ntrip']
    ntrip_cfg['enabled'] = True
    ntrip_cfg['host'] = host
    ntrip_cfg['port'] = port
    ntrip_cfg['mountpoint'] = mountpoint
    ntrip_cfg['username'] = username
    ntrip_cfg['password'] = password
    if 'password_env' in ntrip_cfg:
        del ntrip_cfg['password_env'] 
    
    out_params = {'/um982_rtk_driver': um982_config}
    with open(OUT_PARAMS_FILE, 'w', encoding='utf-8') as f:
        yaml.safe_dump(out_params, f, default_flow_style=False)
        
    print(f"✅ Created parameter file at {OUT_PARAMS_FILE}")

    # --- Generate bash exports ---
    with open(ENV_FILE, "w") as f:
        f.write(f"export FYP_RTK_PARAMS_FILE='{OUT_PARAMS_FILE}'\n")

if __name__ == "__main__":
    main()
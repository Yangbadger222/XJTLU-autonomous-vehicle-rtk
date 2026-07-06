import sys
import socket
import base64
import yaml
import os
import json
import getpass
import argparse

# --- Static NTRIP Configuration ---
HOST = "120.253.239.161"
PORT = 8002
MOUNTPOINT = "RTCM33_GRCEJ"

# --- File Paths ---
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
    parser.add_argument('--logout', action='store_true', help='Clear saved credentials')
    parser.add_argument('--status', action='store_true', help='Check current login status')
    args = parser.parse_args()

    # Handle Status Check
    if args.status:
        if not os.path.exists(CRED_CACHE):
            print("Status: ⚪ Not logged in (No cached credentials found).")
            sys.exit(0)
            
        try:
            with open(CRED_CACHE, 'r') as f:
                creds = json.load(f)
                username = creds.get('username')
                password = creds.get('password')
        except json.JSONDecodeError:
            print("Status: ❌ Error reading cached credentials. Please run 'make ntrip-login'.")
            sys.exit(1)
            
        print(f"Current User: {username}")
        print(f"Server: {HOST}:{PORT} (Mountpoint: {MOUNTPOINT})")
        print("Pinging server...")
        
        # Use a strict 1-second timeout for the status check
        success, msg = test_ntrip(HOST, PORT, MOUNTPOINT, username, password, timeout_sec=1.0)
        if success:
            print("Status: ✅ ACTIVE (Connection successful)")
        else:
            print(f"Status: ❌ EXPIRED / FAILED ({msg})")
            print("Hint: Run 'make ntrip-login' to update your credentials.")
        sys.exit(0)

    # Handle Logout
    if args.logout:
        if os.path.exists(CRED_CACHE):
            os.remove(CRED_CACHE)
            print("Logged out. Cached credentials removed.")
        else:
            print("No cached credentials found.")
        sys.exit(0)

    # Handle Login
    username = None
    password = None

    if os.path.exists(CRED_CACHE):
        try:
            with open(CRED_CACHE, 'r') as f:
                creds = json.load(f)
                c_user = creds.get('username')
                c_pass = creds.get('password')
                
            if c_user and c_pass:
                print(f"Found previous credentials for user: {c_user}")
                choice = input("Do you want to restore this session? [Y/n]: ").strip().lower()
                
                # If they hit Enter (empty string) or type 'y', restore it.
                if choice in ['', 'y', 'yes']:
                    username = c_user
                    password = c_pass
                else:
                    print("\nProceeding with new login...")
        except json.JSONDecodeError:
            pass

    # Prompt if no cache was found or user chose not to restore
    if not username or not password:
        print(f"Connecting to {HOST}:{PORT} (Mountpoint: {MOUNTPOINT})")
        username = input("Username: ").strip()
        password = getpass.getpass("Password: ").strip()

    print("\nTesting connection...")
    success, msg = test_ntrip(HOST, PORT, MOUNTPOINT, username, password, timeout_sec=5.0)
    
    if success:
        print("✅ Connection test successful!")
        with open(CRED_CACHE, 'w') as f:
            json.dump({'username': username, 'password': password}, f)
    else:
        print(f"❌ Connection test failed: {msg}")
        if os.path.exists(CRED_CACHE):
            os.remove(CRED_CACHE)
        sys.exit(1)

    # --- Generate the ROS YAML File ---
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
    ntrip_cfg['host'] = HOST
    ntrip_cfg['port'] = PORT
    ntrip_cfg['mountpoint'] = MOUNTPOINT
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
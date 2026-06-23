import sys
import re
import socket
import base64
import yaml
import os

def test_ntrip(host, port, mountpoint, username, password):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5.0)
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
    print("Please paste the NTRIP account info below.")
    print("When finished pasting, type '==' on a new line and press Enter:")
    
    lines = []
    while True:
        try:
            line = input()
            lines.append(line)
            
            # Combine current lines to check for the double '==' delimiter
            current_text = "\n".join(lines)
            
            # Break if we have two '==' in the text, or if the user manually typed '==' or 'EOF' alone.
            if current_text.count("==") >= 2:
                break
            if line.strip() in ["==", "EOF", "quit", "exit"]:
                break
        except EOFError:
            break
            
    text = "\n".join(lines)
    
    # Parse the text
    username_match = re.search(r"账号[:：]\s*([a-zA-Z0-9]+)", text)
    password_match = re.search(r"密码[:：]\s*([a-zA-Z0-9]+)", text)
    ip_match = re.search(r"IP[:：]\s*([0-9\.]+)", text)
    mountpoint_match = re.search(r"接入点[:：]\s*([A-Za-z0-9_]+)", text)
    
    # Port parsing. Prefer the WGS84 one, usually 8002.
    port_match = re.search(r"端口[:：].*?(\d+)[（\(]WGS84", text)
    if not port_match:
        port_match = re.search(r"端口[:：]\s*(\d+)", text)
        
    if not (username_match and password_match and ip_match and mountpoint_match and port_match):
        print("\nError: Could not parse all required information. Please check the format.")
        sys.exit(1)

    username = username_match.group(1)
    password = password_match.group(1)
    host = ip_match.group(1)
    mountpoint = mountpoint_match.group(1)
    port = int(port_match.group(1))
    
    print(f"\nParsed info:")
    print(f"Host: {host}:{port}")
    print(f"Mountpoint: {mountpoint}")
    print(f"Username: {username}")
    print(f"Password: {password}")
    
    print("\nTesting connection...")
    success, msg = test_ntrip(host, port, mountpoint, username, password)
    if success:
        print("Connection test successful!")
    else:
        print(f"Connection test failed: {msg}")
        choice = input("Do you want to continue anyway? (y/n): ")
        if choice.lower() != 'y':
            print("Aborted.")
            sys.exit(1)

    # Load master params
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
    ntrip_cfg['password'] = ""
    ntrip_cfg['password_env'] = "NTRIP_PASSWORD"
    
    out_params = {'/um982_rtk_driver': um982_config}
    
    out_file = "/tmp/um982_cors.yaml"
    with open(out_file, 'w', encoding='utf-8') as f:
        yaml.safe_dump(out_params, f, default_flow_style=False)
        
    print(f"\nCreated parameter file at {out_file}")
    
    # Write exports to be sourced by the bash wrapper
    with open("/tmp/ntrip_env.sh", "w") as f:
        f.write(f"export NTRIP_PASSWORD='{password}'\n")
        f.write(f"export FYP_RTK_PARAMS_FILE='{out_file}'\n")

if __name__ == "__main__":
    main()
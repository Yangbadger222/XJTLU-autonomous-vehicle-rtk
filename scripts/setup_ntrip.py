#!/usr/bin/env python3
"""Configure and select runtime-only UM982 NTRIP profiles."""

from __future__ import annotations

import argparse
import base64
import getpass
import os
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import shlex
import shutil
import socket
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parent.parent
RUNTIME_CONFIG_ENV = "FYP_NTRIP_RUNTIME_CONFIG_DIR"
RUNTIME_CONFIG_DEFAULT = REPO_ROOT / "runtime-data" / "config"
PASSWORD_ENV = "NTRIP_PASSWORD"


@dataclass(frozen=True)
class Profile:
    key: str
    aliases: tuple[str, ...]
    label: str
    params_filename: str
    baud: int


PROFILES = {
    "standard": Profile(
        key="standard",
        aliases=("standard", "old", "legacy"),
        label="standard / old (NMEA, 115200 baud)",
        params_filename="um982_cors.yaml",
        baud=115200,
    ),
    "mixed": Profile(
        key="mixed",
        aliases=("mixed", "new"),
        label="mixed / new (raw + NMEA, 921600 baud)",
        params_filename="um982_cors_mixed.yaml",
        baud=921600,
    ),
}


def runtime_config_dir() -> Path:
    return Path(os.environ.get(RUNTIME_CONFIG_ENV, RUNTIME_CONFIG_DEFAULT)).expanduser()


def env_file(config_dir: Path) -> Path:
    return config_dir / "um982_cors_env.sh"


def resolve_profile(value: str) -> Profile:
    normalized = value.strip().lower()
    for profile in PROFILES.values():
        if normalized in profile.aliases:
            return profile
    choices = ", ".join("/".join(profile.aliases) for profile in PROFILES.values())
    raise ValueError(f"unknown profile '{value}', choose one of: {choices}")


def profile_path(config_dir: Path, profile: Profile) -> Path:
    return config_dir / profile.params_filename


def profile_root(data: dict[object, object]) -> str:
    for key in data:
        if isinstance(key, str) and key.lstrip("/") == "um982_rtk_driver":
            return key
    raise ValueError("UM982 parameter file has no um982_rtk_driver root")


def load_profile(config_dir: Path, profile: Profile) -> tuple[Path, dict[object, object], dict[str, object]]:
    path = profile_path(config_dir, profile)
    if not path.is_file():
        raise FileNotFoundError(
            f"{profile.label} runtime YAML is missing: {path}. "
            "Run bash scripts/init_runtime_data.sh on the Jetson first."
        )
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"invalid YAML root in {path}")
    root = profile_root(data)
    node = data[root]
    if not isinstance(node, dict) or not isinstance(node.get("ros__parameters"), dict):
        raise ValueError(f"missing ros__parameters in {path}")
    params = node["ros__parameters"]
    ntrip = params.get("ntrip")
    if not isinstance(ntrip, dict):
        ntrip = {}
        params["ntrip"] = ntrip
    return path, data, ntrip


def read_shell_export(path: Path, name: str) -> str:
    if not path.is_file():
        return ""
    prefix = f"export {name}="
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.startswith(prefix):
            continue
        values = shlex.split(line[len(prefix):], posix=True)
        return values[0] if len(values) == 1 else ""
    return ""


def active_profile(config_dir: Path) -> Profile:
    selected = read_shell_export(env_file(config_dir), "FYP_RTK_ACTIVE_PROFILE")
    return resolve_profile(selected) if selected else PROFILES["standard"]


def profile_password(ntrip: dict[str, object], config_dir: Path) -> str:
    direct = str(ntrip.get("password", ""))
    if direct:
        return direct
    return read_shell_export(env_file(config_dir), str(ntrip.get("password_env", PASSWORD_ENV)))


def prompt_value(label: str, default: str = "") -> str:
    suffix = f" [{default}]" if default else ""
    value = input(f"{label}{suffix}: ").strip()
    return value or default


def choose_targets(initial: str | None) -> list[Profile]:
    if initial:
        if initial == "both":
            return [PROFILES["standard"], PROFILES["mixed"]]
        return [resolve_profile(initial)]
    print("\nSelect NTRIP parameter profile:")
    print("  1. standard / old  - NMEA at 115200 baud")
    print("  2. mixed / new     - raw + NMEA at 921600 baud")
    print("  3. both             - update both profiles (recommended)")
    choice = input("Profile [3]: ").strip() or "3"
    mapping = {
        "1": [PROFILES["standard"]],
        "2": [PROFILES["mixed"]],
        "3": [PROFILES["standard"], PROFILES["mixed"]],
    }
    if choice not in mapping:
        raise ValueError("profile must be 1, 2, or 3")
    return mapping[choice]


def choose_active(targets: list[Profile], config_dir: Path) -> Profile:
    if len(targets) == 1:
        return targets[0]
    current = active_profile(config_dir).key
    selected = prompt_value("Active profile after setup (standard/mixed)", current)
    profile = resolve_profile(selected)
    if profile not in targets:
        raise ValueError("active profile must be one of the profiles updated in this setup")
    return profile


def ntrip_request(host: str, port: int, mountpoint: str, username: str, password: str) -> tuple[bool, str]:
    token = base64.b64encode(f"{username}:{password}".encode()).decode()
    request = (
        f"GET /{mountpoint.lstrip('/')} HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        "Ntrip-Version: Ntrip/2.0\r\n"
        "User-Agent: XJTLU-NTRIP-Setup/1.0\r\n"
        f"Authorization: Basic {token}\r\n"
        "Connection: close\r\n\r\n"
    ).encode()
    try:
        with socket.create_connection((host, port), timeout=8.0) as connection:
            connection.sendall(request)
            response = connection.recv(512)
    except OSError as error:
        return False, str(error)
    first_line = response.split(b"\n", 1)[0].rstrip(b"\r").decode("ascii", errors="replace")
    accepted = first_line.startswith("ICY 200") or (
        first_line.startswith("HTTP/") and " 200" in first_line
    )
    return accepted, first_line or "no response"


def write_profile(
    config_dir: Path,
    profile: Profile,
    host: str,
    port: int,
    mountpoint: str,
    username: str,
    backup_stamp: str,
) -> None:
    path, data, ntrip = load_profile(config_dir, profile)
    backup = path.with_name(f"{path.name}.bak.{backup_stamp}")
    shutil.copy2(path, backup)
    ntrip.update(
        {
            "enabled": True,
            "host": host,
            "port": port,
            "mountpoint": mountpoint,
            "username": username,
            "password": "",
            "password_env": PASSWORD_ENV,
            "connect_requires_valid_gga": True,
        }
    )
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    os.chmod(path, 0o600)


def write_environment(config_dir: Path, profile: Profile, password: str, backup_stamp: str) -> None:
    path = env_file(config_dir)
    if path.is_file():
        shutil.copy2(path, path.with_name(f"{path.name}.bak.{backup_stamp}"))
    standard_path = profile_path(config_dir, PROFILES["standard"])
    mixed_path = profile_path(config_dir, PROFILES["mixed"])
    active_path = profile_path(config_dir, profile)
    lines = [
        "# Runtime-only CORS credentials. Do not commit.",
        f"export FYP_RTK_ACTIVE_PROFILE={shlex.quote(profile.key)}",
        f"export FYP_RTK_STANDARD_PARAMS_FILE={shlex.quote(str(standard_path))}",
        f"export FYP_RTK_MIXED_PARAMS_FILE={shlex.quote(str(mixed_path))}",
        f"export FYP_RTK_PARAMS_FILE={shlex.quote(str(active_path))}",
        f"export FYP_UM982_MIXED_PARAMS_FILE={shlex.quote(str(mixed_path))}",
        f"export {PASSWORD_ENV}={shlex.quote(password)}",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")
    os.chmod(path, 0o600)


def setup(config_dir: Path, requested_profile: str | None) -> int:
    targets = choose_targets(requested_profile)
    _, _, existing_ntrip = load_profile(config_dir, targets[0])
    print(f"\nConfiguring: {', '.join(profile.label for profile in targets)}")
    host = prompt_value("Caster host", str(existing_ntrip.get("host", "")))
    port = int(prompt_value("Caster port", str(existing_ntrip.get("port", 2101))))
    mountpoint = prompt_value("Mountpoint", str(existing_ntrip.get("mountpoint", "")))
    username = prompt_value("Username", str(existing_ntrip.get("username", "")))
    password = getpass.getpass("Password: ").strip()
    if not all((host, mountpoint, username, password)):
        raise ValueError("host, mountpoint, username, and password are required")
    success, result = ntrip_request(host, port, mountpoint, username, password)
    if not success:
        print(f"NTRIP test failed: {result}", file=sys.stderr)
        return 1
    active = choose_active(targets, config_dir)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    for profile in targets:
        write_profile(config_dir, profile, host, port, mountpoint, username, stamp)
    write_environment(config_dir, active, password, stamp)
    print(f"NTRIP test succeeded: {result}")
    print(f"Active profile: {active.label}")
    print(f"Runtime config: {profile_path(config_dir, active)}")
    return 0


def login(config_dir: Path, requested_profile: str | None) -> int:
    profile = resolve_profile(requested_profile) if requested_profile else active_profile(config_dir)
    _, _, ntrip = load_profile(config_dir, profile)
    username = prompt_value("Username", str(ntrip.get("username", "")))
    password = getpass.getpass("Password: ").strip()
    if not username or not password:
        raise ValueError("username and password are required")
    host = str(ntrip.get("host", ""))
    port = int(ntrip.get("port", 2101))
    mountpoint = str(ntrip.get("mountpoint", ""))
    success, result = ntrip_request(host, port, mountpoint, username, password)
    if not success:
        print(f"NTRIP test failed: {result}", file=sys.stderr)
        return 1
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    write_profile(config_dir, profile, host, port, mountpoint, username, stamp)
    write_environment(config_dir, profile, password, stamp)
    print(f"NTRIP login succeeded: {result}")
    return 0


def use_profile(config_dir: Path, requested_profile: str) -> int:
    profile = resolve_profile(requested_profile)
    _, _, ntrip = load_profile(config_dir, profile)
    password = profile_password(ntrip, config_dir)
    if not password or not str(ntrip.get("username", "")):
        raise ValueError(f"{profile.label} has no saved NTRIP credentials; run make ntrip-setup")
    write_environment(config_dir, profile, password, datetime.now().strftime("%Y%m%d-%H%M%S"))
    print(f"Active NTRIP profile: {profile.label}")
    print(f"Launch parameter file: {profile_path(config_dir, profile)}")
    return 0


def status(config_dir: Path) -> int:
    profile = active_profile(config_dir)
    path, _, ntrip = load_profile(config_dir, profile)
    password = profile_password(ntrip, config_dir)
    host = str(ntrip.get("host", ""))
    port = int(ntrip.get("port", 2101))
    mountpoint = str(ntrip.get("mountpoint", ""))
    username = str(ntrip.get("username", ""))
    print(f"Active profile: {profile.label}")
    print(f"Parameter file: {path}")
    print(f"UM982 baud: {profile.baud}")
    print(f"NTRIP endpoint: {host}:{port}/{mountpoint}")
    print(f"Username: {username or '[not configured]'}")
    if not password:
        print("NTRIP status: NOT CONFIGURED")
        return 1
    success, result = ntrip_request(host, port, mountpoint, username, password)
    print(f"NTRIP status: {'ACTIVE' if success else 'FAILED'} ({result})")
    return 0 if success else 1


def logout(config_dir: Path) -> int:
    profile = active_profile(config_dir)
    path, data, ntrip = load_profile(config_dir, profile)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    shutil.copy2(path, path.with_name(f"{path.name}.bak.{stamp}"))
    ntrip.update({"enabled": False, "username": "", "password": ""})
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    os.chmod(path, 0o600)
    write_environment(config_dir, profile, "", stamp)
    print(f"NTRIP credentials cleared for {profile.label}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--setup", action="store_true", help="configure endpoint and credentials")
    action.add_argument("--login", action="store_true", help="replace credentials for one profile")
    action.add_argument("--status", action="store_true", help="test the active profile")
    action.add_argument("--logout", action="store_true", help="clear active profile credentials")
    action.add_argument("--use-profile", metavar="PROFILE", help="select standard/old or mixed/new")
    parser.add_argument("--profile", help="target standard/old, mixed/new, or both during setup")
    args = parser.parse_args()
    config_dir = runtime_config_dir()
    try:
        if args.setup:
            return setup(config_dir, args.profile)
        if args.status:
            return status(config_dir)
        if args.logout:
            return logout(config_dir)
        if args.use_profile:
            return use_profile(config_dir, args.use_profile)
        return login(config_dir, args.profile)
    except (FileNotFoundError, ValueError, OSError, yaml.YAMLError) as error:
        print(f"NTRIP configuration error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

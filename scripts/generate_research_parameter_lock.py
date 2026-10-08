#!/usr/bin/env python3
"""Generate shared physical constants from the approved source-derived lock."""
import hashlib
import json
import argparse
import re
import math
import subprocess
import yaml
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="fail if generated constants differ")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    source = root/"audit/vehicle_baseline/VEHICLE_PARAMETER_LOCK.json"
    payload = json.loads(source.read_text())
    if payload["source_commit"] != "e54c6afbcb5a58db22d7c468085a87d658b0b932":
        raise SystemExit("approved vehicle source identity differs")
    locked = {item["name"]: item["value"] for item in payload["parameters"]}
    vmax,vmin,acc,dec = (locked[name] for name in ("corridor.smoother.max_velocity",
        "corridor.smoother.min_velocity","corridor.smoother.max_accel","corridor.smoother.max_decel"))
    values = dict(max_speed_mps=vmax[0],min_speed_mps=vmin[0],max_yaw_rate_rps=vmax[2],
                  max_accel_mps2=acc[0],max_decel_mps2=-dec[0],
                  max_yaw_accel_rps2=acc[2],max_yaw_decel_rps2=-dec[2],max_lateral_accel_mps2=locked["corridor.guard.turn_product_limit"])
    firmware = {key:locked['firmware.source.source_values.'+key] for key in
                ('radius_m','track_m','gear_ratio','radps_to_rpm','max_motor_rpm')}
    # Conservative optimizer envelope, not a measured skid-steer ability.
    # At every 0<v<=Vmax this fixed cap is inside both original w and v*w
    # limits. The original speed-dependent bounds are still checked at runtime.
    execution_profile = dict(max_curvature_1pm=min(values['max_yaw_rate_rps']/values['max_speed_mps'],
        values['max_lateral_accel_mps2']/values['max_speed_mps']**2),
        max_lateral_speed_mps=0.05, max_jerk_mps3=2*values['max_accel_mps2']/1.0)
    notes_path = 'docs-CN/hardware_spec.md'
    original_notes = subprocess.check_output(['git','show',payload['source_commit']+':'+notes_path],cwd=root)
    if (root/notes_path).read_bytes() != original_notes:
        raise SystemExit("recorded MID360 source notes differ from approved baseline")
    height_match = re.search(r'安装高度 \(相对地面\) \| \*\*([0-9.]+) m\*\*',original_notes.decode())
    if not height_match:
        raise SystemExit("recorded MID360 mounting height unavailable")
    fov_match=re.search(r'\| FOV \| ([0-9.]+)x([0-9.]+) 度 \|',original_notes.decode())
    if not fov_match:raise SystemExit('recorded MID360 FOV unavailable')
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    footprint = tuple(tuple(point) for point in locked["vehicle.corridor_footprint_xy"])
    master_path = root/"src/bringup/config/master_params.yaml"
    protected = dict(line.split(None, 1)[::-1] for line in
        (root/"audit/vehicle_baseline/PROTECTED_FILES.sha256").read_text().splitlines())
    if hashlib.sha256(master_path.read_bytes()).hexdigest() != protected["src/bringup/config/master_params.yaml"]:
        raise SystemExit("protected master source differs; refusing to regenerate stop confirmation")
    master = yaml.safe_load(master_path.read_text())["/rtk_map_odom_corrector"]["ros__parameters"]
    lidar_params = yaml.safe_load(master_path.read_text())["/fastlio2"]["lio_node"]["ros__parameters"]
    ground_reference = dict(lidar_height_m=float(height_match.group(1)),
                            lidar_in_imu_m=tuple(lidar_params['t_il']))
    stopped = (master["stopped_linear_rate_mps"], math.radians(master["stopped_yaw_rate_degps"]),
               master["stopped_confirmation_s"])
    python = root/"src/research_runtime/research_runtime/physical_parameter_lock.py"
    python_text = (f'"""Generated from approved parameter lock; SHA256 {digest}."""\n'
        +f"PHYSICAL_LIMITS = {values!r}\n"
        +f"LOCKED_FOOTPRINT = {footprint!r}\n"
        +f"STOP_CONFIRMATION = {stopped!r}\n"
        +f"STOP_CONFIRMATION_SOURCE_SHA256 = {hashlib.sha256(master_path.read_bytes()).hexdigest()!r}\n"
        +f"AUTHORITY_HEARTBEAT_TIMEOUT_S = {locked['authority.heartbeat_timeout_s']!r}\n"
        +f"FIRMWARE_COMMAND_MODEL = {firmware!r}\n"
        +f"RESEARCH_EXECUTION_PROFILE = {execution_profile!r}\n"
        +f"MID360_GROUND_REFERENCE = {ground_reference!r}\n"
        +f"MID360_MOUNTING_NOTES_SHA256 = {hashlib.sha256(original_notes).hexdigest()!r}\n"
        +'''\ndef require_locked_motion_parameters(actual):
    import math
    for key, expected in PHYSICAL_LIMITS.items():
        value = actual.get(key)
        if not isinstance(value, (int,float)) or not math.isfinite(value) or abs(value-expected) > 1e-12:
            raise ValueError(f"protected corridor parameter override rejected: {key}")
''')
    header = root/"src/research_interfaces/include/research_interfaces/vehicle_parameter_lock.hpp"
    header_text = (f"// Generated from approved vehicle parameter lock; SHA256 {digest}\n#pragma once\n"
        +"namespace research_vehicle_lock {\n"+"".join(f"inline constexpr double {key} = {value:.17g};\n"
                                                      for key,value in values.items())
        +"namespace firmware_command {\n"+"".join(f"inline constexpr double {key} = {value:.17g};\n"
                                                      for key,value in firmware.items())+"}\n}\n")
    # These are new research files. Original baseline YAML/launch files remain
    # byte-protected; no physical value is maintained by hand in a second set.
    for name in ("ego_vehicle_adapter","research_safety_bridge"):
        config=root/f"src/bringup/config/{name}.yaml"
        text=config.read_text()
        actual=yaml.safe_load(text)[name]["ros__parameters"]
        if args.check:
            if any(actual.get(key)!=value for key,value in values.items()):
                raise SystemExit(f"research YAML physical values differ from lock: {name}")
            if name=="research_safety_bridge" and actual.get("footprint_xy")!=[v for point in footprint for v in point]:
                raise SystemExit("research footprint differs from approved lock")
        else:
            for key,value in values.items():
                text,count=re.subn(rf"^(    {key}:) .*$",rf"\1 {float(value)!r}",text,flags=re.MULTILINE)
                if count!=1:raise SystemExit("one physical parameter row required: "+key)
            config.write_text(text)
        for key,value in execution_profile.items():
            if key == 'max_jerk_mps3' and name == 'research_safety_bridge':
                continue
            if args.check and actual.get(key) != value:
                raise SystemExit(f"derived research execution profile differs: {name}.{key}")
            if not args.check:
                text,count=re.subn(rf"^(    {key}:) .*$",rf"\1 {float(value)!r}",text,flags=re.MULTILINE)
                if count!=1:raise SystemExit("one algorithm profile row required: "+key)
                config.write_text(text)
        if name=="ego_vehicle_adapter":
            radius=max(math.sqrt(x*x+y*y) for x,y in footprint)
            if args.check and actual["inflate_radius_m"]!=radius:raise SystemExit("planner footprint circle differs from source polygon")
            if not args.check:
                text=re.sub(r"^(    inflate_radius_m:) .*$",rf"\1 {radius!r}",text,flags=re.MULTILINE)
                config.write_text(text)
    cloud=root/"src/bringup/config/super_lio_cloud_frame.yaml"
    cloud_text=cloud.read_text();cloud_values=yaml.safe_load(cloud_text)["super_lio_cloud_frame_adapter"]["ros__parameters"]
    for key,value in list(zip(("obstacle_min_z_m","obstacle_max_z_m"),locked["cloud.nav2_obstacle_height_window"]))+list(zip(("publish_min_z_m","publish_max_z_m"),locked["cloud.publish_height_window"])):
        if args.check and cloud_values[key]!=value:raise SystemExit("cloud height window differs from original lock")
        if not args.check:cloud_text=re.sub(rf"^(    {key}:) .*$",rf"\1 {float(value)!r}",cloud_text,flags=re.MULTILINE)
    if not args.check:cloud.write_text(cloud_text)
    observation=root/'src/bringup/config/active_observation.yaml'
    observation_text=observation.read_text()
    observation_actual=yaml.safe_load(observation_text)['active_observation']['ros__parameters']
    for key,value in dict(max_curvature_1pm=execution_profile['max_curvature_1pm'],
                         sensor_fov_rad=math.radians(float(fov_match.group(1)))).items():
        if args.check and observation_actual[key]!=value:raise SystemExit('derived observation setting differs: '+key)
        if not args.check:
            observation_text,count=re.subn(rf'^(    {key}:) .*$',rf'\1 {float(value)!r}',observation_text,flags=re.MULTILINE)
            if count!=1:raise SystemExit('one observation setting required: '+key)
    if not args.check:observation.write_text(observation_text)
    if args.check:
        for path, expected in ((python, python_text), (header, header_text)):
            if not path.exists() or path.read_text() != expected:
                raise SystemExit(f"generated physical lock differs: {path}")
        print("Shared Python/C++ physical constants match the approved lock")
    else:
        header.parent.mkdir(parents=True,exist_ok=True)
        python.write_text(python_text)
        header.write_text(header_text)
        print("Generated Python/C++ constants from approved corridor launch values")


if __name__ == "__main__":
    main()

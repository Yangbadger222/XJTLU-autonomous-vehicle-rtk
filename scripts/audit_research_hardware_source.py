#!/usr/bin/env python3
"""Read original firmware/model values; never equate source to flashed calibration."""
import json,re,hashlib,argparse,math
from pathlib import Path

def audit(root):
    path=root/'src/firmware/rm_c_board/Core/Src/Motor_Speed_pid.c'
    text=path.read_text(errors='replace')
    def scalar(pattern):
        match=re.search(pattern,text)
        if not match:raise ValueError('firmware field unavailable: '+pattern)
        return float(match[1])
    values={'track_m':scalar(r'float C\s*=\s*([\d.]+)'),
            'radius_m':scalar(r'float r\s*=\s*([\d.]+)'),
            'max_motor_rpm':scalar(r'#define MAX_SPEED_RPM\s+([\d.]+)'),
            'gear_ratio':scalar(r'set_spdL =[^;]+\* ([\d.]+);'),
            'radps_to_rpm':scalar(r'\(2 \* r\) \* ([\d.]+)'),
            'brake_stop_rpm':scalar(r'#define BRAKE_STOP_RPM\s+([\d.]+)'),
            'brake_low_speed_rpm':scalar(r'#define BRAKE_LOW_SPEED_RPM\s+([\d.]+)'),
            'brake_current_limit':scalar(r'#define BRAKE_CURRENT_LIMIT\s+([\d.]+)'),
            'brake_low_speed_current_limit':scalar(r'#define BRAKE_LOW_SPEED_CURRENT_LIMIT\s+([\d.]+)'),
            'brake_damping_kp':scalar(r'#define BRAKE_DAMPING_KP\s+([\d.]+)'),
            'brake_current_step':scalar(r'#define BRAKE_CURRENT_STEP\s+([\d.]+)')}
    active=re.sub(r'/\*.*?\*/|//[^\n]*','',text,flags=re.S)
    feedback=active.split('void MOTORrpm2vw',1)[1]
    feedback_values={key:float(re.search(pattern,feedback)[1]) for key,pattern in
        [('track_m',r'const float c\s*=\s*([\d.]+)'),('radius_m',r'const float r\s*=\s*([\d.]+)'),
         ('gear_ratio',r'left_motor_speed/([\d.]+)f'),('radps_to_rpm',r'left_motor_speed/[\d.]+f/([\d.]+)f')]}
    pid_args=re.search(r'f_param_init\(&motor_pid\[i\],\s*PID_Speed,([^;]+)\);',active)[1]
    pid_numbers=[float(s.strip()) for s in pid_args.split(',')]
    pid_values=dict(zip(('max_output','integral_limit','deadband','period','max_error','initial_target','kp','ki','kd'),pid_numbers))
    lock=json.loads((root/'audit/vehicle_baseline/VEHICLE_PARAMETER_LOCK.json').read_text())
    locked={p['name']:p['value'] for p in lock['parameters']};v,_,w=locked['corridor.smoother.max_velocity']
    worst_rpm=(v+w*values['track_m']/2)/values['radius_m']*values['radps_to_rpm']*values['gear_ratio']
    checks={'entire_v_w_cap_rectangle_below_source_motor_rpm':worst_rpm<values['max_motor_rpm'],
            'four_motor_sign_and_grouping':all(s in text for s in ['motor_pid[0].target = set_spdL','motor_pid[1].target = set_spdL','motor_pid[2].target = -set_spdR','motor_pid[3].target = -set_spdR'])}
    return {'status':'PASS' if all(checks.values()) else 'FAIL','source':str(path.relative_to(root)),
            'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'source_values':values,
            'command_equations':['rpmL=(v-w*C/2)/r*9.55*19.2','rpmR=(v+w*C/2)/r*9.55*19.2','motor_targets=[rpmL,rpmL,-rpmR,-rpmR]'],
            'feedback_source_values':feedback_values,'pid_source_values':pid_values,
            'worst_case_command_motor_rpm':worst_rpm,'checks':checks,
            'discrepancies':['firmware command track .46 differs from feedback/URDF .50',
                'firmware command gear 19.2 differs from feedback 19.0',
                'firmware/URDF radius .10 differs from hardware-spec claimed diameter .085'],
            'scope':'source contract proof only; no firmware edit/flash and no claim that these are physically measured or currently flashed',
            'pending':['flashed firmware identity; measured six-wheel geometry/slip','physical current/brake/KEY/PS2 verification; no source wheel acceleration limit supplied']}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=audit(a.root);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));raise SystemExit(0 if result['status']=='PASS' else 1)

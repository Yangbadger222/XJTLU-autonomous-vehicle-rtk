#!/usr/bin/env python3
"""Compile native probes against the actual colcon-produced EGO libraries."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess

p = argparse.ArgumentParser()
p.add_argument('--repo', type=Path, required=True)
p.add_argument('--workspace', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
build = a.workspace/'build/ego_planner'
flags = {}
for line in (build/'CMakeFiles/motion_plan.dir/flags.make').read_text().splitlines():
    if line.startswith('CXX_'):
        key,value = line.split('=',1)
        flags[key.strip()] = shlex.split(value)
link = shlex.split((build/'CMakeFiles/motion_plan.dir/link.txt').read_text())
a.output.parent.mkdir(parents=True,exist_ok=True)
cases=[]
for name in ('boundary','turn_gradient','turn_bounds','grid_contract','measured_start'):
    source = a.repo/f'scripts/validate_ego_{"bspline_boundary" if name == "boundary" else name}.cpp'
    obj = a.output.parent/f'ego_native_{name}.o'
    executable = a.output.parent/f'ego_native_{name}'
    compile_command = [link[0]]+flags['CXX_DEFINES']+flags['CXX_INCLUDES']+flags['CXX_FLAGS']+['-c',str(source),'-o',str(obj)]
    compiled = subprocess.run(compile_command,cwd=build,capture_output=True,text=True)
    command = [str(obj) if token.endswith('trajectory_publisher.cpp.o') else token for token in link]
    command[command.index('-o')+1] = str(executable)
    linked = subprocess.run(command,cwd=build,capture_output=True,text=True) if compiled.returncode == 0 else None
    executed = subprocess.run([str(executable)],capture_output=True,text=True) if linked and linked.returncode == 0 else None
    cases.append({'name':name,'compile_exit':compiled.returncode,'compile_stderr':compiled.stderr,
                  'link_exit':linked.returncode if linked else None,'link_stderr':linked.stderr if linked else None,
                  'run_stderr':executed.stderr if executed else None,'run_exit':executed.returncode if executed else None,'output':executed.stdout if executed else None,
                  'status':'PASS' if executed and executed.returncode == 0 else 'FAIL'})
result={'scope':'native probes linked to the actual patched upstream core','cases':cases,
        'status':'PASS' if all(c['status']=='PASS' for c in cases) else 'FAIL'}
a.output.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
raise SystemExit(0 if result['status']=='PASS' else 1)

import json,sys,time,subprocess,runpy
from pathlib import Path
log=Path('/research-ws/qualification/ground-signal-arm-cli-timing.jsonl')
log.open('x').close()
actual=subprocess.run
def measured(*args,**kwargs):
    cmd=args[0] if args else kwargs.get('args')
    start=time.monotonic()
    with log.open('a') as f:f.write(json.dumps({'event':'start','command':cmd,'monotonic_s':start})+'\n')
    try:
        result=actual(*args,**kwargs)
    except BaseException as exc:
        with log.open('a') as f:f.write(json.dumps({'event':'raised','command':cmd,'wall_s':time.monotonic()-start,'exception_type':type(exc).__name__})+'\n')
        raise
    with log.open('a') as f:f.write(json.dumps({'event':'end','command':cmd,'wall_s':time.monotonic()-start,'exit':result.returncode})+'\n')
    return result
subprocess.run=measured
sys.argv=['/vehicle-research/scripts/validate_research_launch.py']+sys.argv[1:]
runpy.run_path(sys.argv[0],run_name='__main__')

import ast,json,time
from pathlib import Path
from types import SimpleNamespace
source=Path('/dev/shm/codex-shadow-delivery/scripts/validate_restricted_policy_ros.py')
trial=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='trial')
functions=[n for n in trial.body if isinstance(n,ast.FunctionDef) and n.name in ('plan_cb','capture_context')]
contexts=[];inputs={'state':[0.,0.,0.,0.,0.],'grid':{'resolution':.3,'origin':[-1.,-1.],'width':1,'height':1,'cells':[-1]}}
namespace={'json':json,'time':time,'planning':{},'failures':[],'inputs':inputs,'contexts':contexts,'context_times':{},'input_receipts':{},'current_version':['v1'],'session_id':'contract-only','start':time.monotonic()}
exec(compile(ast.Module(body=functions,type_ignores=[]),str(source),'exec'),namespace)
msg=SimpleNamespace(failure_reason='road_reference_unavailable_or_unconfirmed')
namespace['plan_cb'](msg)
assert len(contexts)==1,'Missing denial context for actual road_reference_unavailable_or_unconfirmed path'
assert contexts[0]['reason']==msg.failure_reason
assert contexts[0]['map_version']=='v1'
assert contexts[0]['inputs']['grid']['cells']==[-1]
inputs['grid']['cells'][0]=0
assert contexts[0]['inputs']['grid']['cells']==[-1],'Snapshot mutated after callback'
print('PASS: actual plan callback preserves incomplete reference denial and immutable measured context')

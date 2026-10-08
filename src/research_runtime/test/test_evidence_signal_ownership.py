"""Exercise actual research main while a message take is interrupted by a signal."""
import signal
from types import SimpleNamespace
import pytest
import ast
from pathlib import Path
from types import ModuleType

def _production_entry(filename, package="active_road_mapping"):
    # Execute the actual source main AST, without importing ROS message libs.
    source=Path(__file__).resolve().parents[2]/package/package/filename
    main=next(node for node in ast.parse(source.read_text()).body
              if isinstance(node,ast.FunctionDef) and node.name=="main")
    module=ModuleType(filename);module.rclpy=None;module.signal=signal
    exec(compile(ast.Module(body=[main],type_ignores=[]),str(source),"exec"),module.__dict__)
    return module

ENTRIES=[(_production_entry("evidence_node.py"),"ActiveRoadEvidenceNode"),
         (_production_entry("map_node.py"),"ActiveRoadMapNode"),
         (_production_entry("observation_node.py"),"ActiveObservationNode"),
         (_production_entry("adapter_node.py", "super_lio_vehicle_adapter"), "SuperLioVehicleAdapter")]

@pytest.mark.parametrize("module,class_name", ENTRIES)
@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
def test_message_take_keeps_context_alive_until_owned_node_cleanup(monkeypatch, signum, module, class_name):
    events=[]
    previous={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
    runtime=SimpleNamespace(alive=False)
    def shutdown():events.append(("shutdown",runtime.alive));runtime.alive=False
    def init(**kwargs):
        runtime.alive=True
        if kwargs.get("signal_handler_options") != "NO":
            # Default rclpy closes the context asynchronously on these signals.
            for s in previous:signal.signal(s,lambda *_:shutdown())
    def take(*_args,**_kwargs):
        signal.raise_signal(signum)
        if not runtime.alive:
            raise RuntimeError("take_message: context closed during conversion")
        events.append(("take_completed",runtime.alive))
    class Node:
        def destroy_node(self):events.append(("destroy",runtime.alive))
    monkeypatch.setattr(module,"rclpy",SimpleNamespace(init=init,ok=lambda:runtime.alive,
        spin=take,spin_once=take,shutdown=shutdown))
    monkeypatch.setattr(module,"SignalHandlerOptions",SimpleNamespace(NO="NO"),raising=False)
    monkeypatch.setattr(module,class_name,Node,raising=False)
    try:
        module.main()
        assert events==[("take_completed",True),("destroy",True),("shutdown",True)]
        assert all(signal.getsignal(s)==previous[s] for s in previous)
    finally:
        for s,h in previous.items():signal.signal(s,h)

@pytest.mark.parametrize("module,class_name", ENTRIES)
def test_normal_runtime_error_is_not_swallowed(monkeypatch, module, class_name):
    runtime=SimpleNamespace(alive=False);events=[]
    previous={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
    def init(**_kwargs):runtime.alive=True
    def fail(*_args,**_kwargs):raise RuntimeError("ordinary subscription failure")
    def shutdown():events.append(("shutdown",runtime.alive));runtime.alive=False
    class Node:
        def destroy_node(self):events.append(("destroy",runtime.alive))
    monkeypatch.setattr(module,"rclpy",SimpleNamespace(init=init,ok=lambda:runtime.alive,
        spin=fail,spin_once=fail,shutdown=shutdown))
    monkeypatch.setattr(module,"SignalHandlerOptions",SimpleNamespace(NO="NO"),raising=False)
    monkeypatch.setattr(module,class_name,Node,raising=False)
    try:
        with pytest.raises(RuntimeError,match="ordinary subscription failure"):module.main()
        assert events==[("destroy",True),("shutdown",True)]
        assert all(signal.getsignal(s)==previous[s] for s in previous)
    finally:
        for s,h in previous.items():signal.signal(s,h)

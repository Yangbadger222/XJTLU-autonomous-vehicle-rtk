## 2026.10.08

### Operator consent, cockpit and independent raw-bag validation

#### File
- `src/research_runtime/`, `src/research_interfaces/`, `src/active_road_mapping/`
- `src/bringup/launch/system_active_road_research.launch.py`
- `scripts/catalog_research_bags.py`, `scripts/validate_console_*`, `scripts/validate_raw_transport_probe_ros.py`
- `audit/optimization_v2/`, research reports and CN/EN cockpit/command documentation

#### Change
Added a loopback cockpit, typed ordered consent and registered-task service. Browser cannot change environment, actuator configuration, physical parameters or directly write velocity. Transport loss and competing consent latch; RTK loss still stops even healthy LIO. Actual paused-clock red at f8 retained a .216m/s final wire; SystemClock arbitration in the original headerless guard makes the ce fault-phase final five wire frames zero without editing original source/limits.

#### Reason
Provide explicit inspectable human task consent and independent data identity. Prevent continued renewal during state-read loss, automatic transport-recovery rearm, frozen stop arbitration under paused bag clocks, or interpreting derived bags/monitor backpressure as independent estimator trials. Actual software and policy runs retain provenance and failures.

#### Effect
Read-only122 metadata entries yield3 independent raw CDR streams. Two more full1x sequential FAST/Super trials retain UNKNOWN source health and frozen08d binary provenance. The actual simultaneous receiver probe demonstrates monitor backpressure: deep3975/3975, shallow3271; no estimator packet-loss inference. Actual HTTP raw start/pause/resume/owned-stop passed; SIGSTOP resume pacing remains a stated limitation.

ce clean SDK/all14 Humble packages built in2m51s. Current128 portable tests,35 final-PTY faults, paused-clock stop,7HTTP cases, actual CUA GET outage, default entry and122 source/parsed/runtime checks passed. Recovery never automatically enables motion. Desktop/tablet show no horizontal overflow. Delivered default replay preview has actuator/mission disabled and no player started.

Six new60s policy tasks passed protocol but reached0/6 goals; benefit remainsFAIL. The earlier45s cohort is retained, not mixed into60s comparisons. Policy shares foundation/perception/safety, without truth input. Synthetic assumptions do not constitute physical acceptance. Concrete Jetson, calibration/health, camera/ground/prior, braking/slip/wheels and hardware e-stop gates remainPENDING. No firmware flash, physical driving, asset deletion, production merge or force push.

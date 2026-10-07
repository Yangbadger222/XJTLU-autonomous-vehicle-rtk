# Current finite optimization and cockpit evidence

Current full SDK/14-package source: ce4641836c265432c74086669fca8dbd6362b7d7. Clean build log is unabridged (including nine packages with stderr warnings); source-provenance hashes the actual current frozen EGO sources and installed console. There is no Jetson or physical actuator evidence here.

- current-serial-faults/current-paused-clock/current-console-http/current-entry: ce current repeat, actual original serial PTY; 35 faults, paused clock, seven HTTP cases, default entry/overrides.
- browser-acceptance/current-before/current-outage/current-proxy/current-verified: ce actual CUA browser. Synthetic HMI fixture. Proxy action names contain no authentication headers. POST stop during GET outage reached backend and was rejected after lease expiration; no heartbeat renewal, final fault-phase wire zeros, recovery stays latched.
- console-raw-controls: actual current installed console, sealed original morning prefix, start/pause/resume/stop; no motion permit. Test script was added after clean build, does not change installed sources. SIGSTOP resume can catch up scheduling time; this is not a strict pacing or EOF performance test.
- paired-07-06-18/paired-14-34-13: new full sequential 1x estimator trials using frozen 08d1835 binaries. Health UNKNOWN; FAST is not truth. IMU received is the shallow Python monitor count, not estimator consumption/loss.
- raw-transport-monitor: actual simultaneous shallow/deep receiver prefix diagnosis; deep 3975/3975, shallow3271. Monitor-only remediation, no algorithm/input configuration changes.
- sealed-bag-catalog: 122 entries / three independent full raw CDR inputs; metadata and storage sealed. Identical derived recordings are not new trials.
- policy-comparison: actual ed4b7f6 full-build six trials, budget60s, protocol PASS / benefit not established. Earlier45s dataset remains remote, not relabelled60s. The same-value consent alias/UI/monitor change in ce is not a policy re-execution.
- final-ego-straight/arc: historical 1f110dc actual core/PT Y closed loops. Other final-* and console-http-final/browser-serve are explicit earlier1f components; not current ce repeats.
- paused-clock-before: actual f8 red, last original wire remains nonzero; current-paused-clock is green. Original guard SystemClock fixes stop arbitration; original serial wall timer only logs.
- browser-proxy-outage: historical ed4 GET outage had no direct in-outage POST check. Current browser-acceptance supplies that missing test; historical assertion is marked NOT_RUN.
- operator-preview: current default replay domain100, actuator/mission false, no bag started. This preview is intentionally active, all finite tests have ended.

Reports archive v1 in audit/delivery_v1. Never interpret old results as current binaries. Simulation assumptions and pending physical gates are in RESULTS and the delivery report. Full logs/odom, negative45s runs, synthetic GeoTIFF fixtures and build products remain task-local remote assets.

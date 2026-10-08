# Shadow sensor ingress qualification

This optional tool prepares a finite sensor-only shadow run on the same Humble host as the existing sensor publishers. The default research launch remains the previously qualified 16-node entry. The tool creates zero source-domain writers and forwards only seven fixed sensor types to a distinct, explicit research domain. It does not forward TF, old LIO outputs, commands, authority, consent or clock.

Use the stdlib supervisor `scripts/run_shadow_sensor_ingress.py` as the public entry. It reserves a new UUID receipt before loading the native executable and records native exit, stderr, exception, signal and wall timeout. An existing receipt is refused and preserved. A native direct invocation can exit before main when a ROS shared library fails; it is not the public acceptance entry. `COMPLETED` describes finite process termination, including an empty input run, and grants no health or motion acceptance.

The Humble native serialized callback checks each sample's nonzero publisher GID against the unique discovered publisher and expected type before forwarding. This is an operational writer check; deployment identity, authentication and transport security remain separate target checks. The source node also has Humble's internal parameter-event subscription. Its ROS logging and parameter-event publishers and parameter services are disabled. The output node has exactly the seven sensor writers.

The input uses best-effort, volatile depth 10; output uses reliable, volatile depth 10. This bounded setting belongs to the new research tool. Actual recorded GNSS bursts exposed losses at depth 1, whose failed receipt is retained. The tool never restamps or repeats cached sensor samples. Acquisition freshness and all original downstream safety gates still apply.

Build only in a separate installation after the qualified core install is available:

```bash
source /opt/ros/humble/setup.bash
source /absolute/research-core/install/setup.bash
cmake -S /absolute/research-source/tools/shadow_sensor_ingress \
  -B /absolute/shadow-build -DCMAKE_BUILD_TYPE=Release
cmake --build /absolute/shadow-build --parallel 1
```

The dependencies are rclcpp, the seven declared message packages, ament_cmake and OpenSSL Crypto. This auxiliary CMake target is separate from the main SDK/14 package allowlist and is not part of the default launch.

On a host with independently verified source and unused research domain IDs, invoke the supervisor with explicit domains and a fresh absolute output path. Keep the research navigation entry's sensor drivers, mission and actuator disabled:

```bash
ROS_LOCALHOST_ONLY=1 python3 /absolute/research-source/scripts/run_shadow_sensor_ingress.py \
  --executable /absolute/shadow-build/research_shadow_sensor_ingress \
  --source-domain 0 --research-domain 118 --duration-s 60 \
  --output /absolute/research-results/shadow-attempt-unique.json
```

Those domain numbers are an example, not verified vehicle deployment values. localhost-only means both sides are on the same host. No target run has occurred. Obtain the actual domain, publisher identities, launch overlays and timing from the target before using the example. The original RTK-loss stop protection applies even when LIO is healthy.

For source-bound isolation qualification, run `validate_shadow_sensor_ingress_ros.py` with explicit `--launcher`, `--executable`, `--source-root` and `--output`. It uses only analytical fixtures in domains 130/131. The 17 gates pass, including the actual DDS queue-order counterexample: a rival sample is queued, the rival exits, discovery falls to one publisher, and that old sample is rejected using its own GID. Startup exceptions, pre-main missing-RMW failure, signal termination and prior-receipt preservation are tested. The complete native received CDR stream and target stream have equal counts and per-topic SHA256 chains.

For existing-data qualification, run `validate_shadow_sensor_bag_replay.py` with those four paths plus `--bag` and `--duration-s 20`. It uses lab domains 134/135 and explicitly reports PREFIX, not full EOF. July21 PM has six recorded allowlist topics; /rtk/health is absent. All 12 gates pass. Observed delivery is 214 LiDAR, 4007 IMU, 225 fix, 450 heading, 898 NMEA and 23 status messages, all 100%. The native full CDR chains match target reception, and every target original payload matches the direct source observation. RMW tail alignment is recorded separately; message fields and original serialized bytes remain unchanged. The bag actually publishes old TF, FAST-LIO odometry and body cloud into the lab source domain; each has 199 source observations and zero target observations.

These results qualify finite Humble sensor ingress. Target ARM64 startup/performance, physical sensor freshness, registered road evidence, deployment/STM identity, actuator stops and physical driving remain unqualified. Current ground replay still has zero confirmed vehicle-width strips, and current six-policy episodes still reach 0/2 goals per strategy. This tool does not change those results.

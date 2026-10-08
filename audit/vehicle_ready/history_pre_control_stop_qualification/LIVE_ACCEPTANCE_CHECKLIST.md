# Conditions before on-site acceptance

This checklist is pending; no new-stack physical motion or firmware flashing has occurred. Recorded MID360 internal IMU/mounting/factory lever, STM source command geometry, caps and bag response are available and used. They are not wholesale missing data.

Completed software gates include the generated nonzero default profile, source STM motor-target checks, explicit stationary rotation, three default arc loops, one90-degree actual EGO recovery loop,22 installed ground/evidence ROS fixtures and47 final-wire rotation faults. Original caps, stop confirmation and RTK loss protection remain unchanged. Native localization still has three full raw replay receipts. These are scoped software results.

Live motion is still denied until terrain/support qualification is resolved: the actual July15 full raw replay produces2850 valid LIO/ground acquisitions but zero supported cells/road strips. A current30-second density diagnostic sees22–76 floor-band returns per sampled frame and at most4/16 required subtiles per cell. Current geometry must not be advertised as a measured drivable surface. Timestamp/TF/PCL integration fixes do not turn this negative result into PASS. Read-only actuator-disabled target shadow can examine this failure safely; it does not grant live motion.

Target preparation: identify the running Jetson deployment and actual CLI/environment/generated launch overlays read-only, preserve its production checkout/install and current assets, compare original source/parsed/effective parameters, compile the submitted research commit in a separate worktree with one worker, run actuator-disabled shadow. Recorded Jetson100.79.128.21 SSH timed out; authorized non-Jetson100.88.131.52 is used for Humble/bags. No timeout implies missing mounting/chassis records.

On-site human tests, after software/target prerequisites pass:

- Confirm the actual flashed STM identity against the preserved source and recorded command/feedback formula differences; retain wheel diameter/virtual-radius discrepancies rather than silently rewriting either.
- Check KEY, PS2 loss/manual takeover, X zero-current/coasting, B active brake and physical estop at the real final actuator; PTY zeros do not prove brake/estop physics.
- Verify current stopping distance, latency, footprint sweep, terrain support assumptions and slip under the preserved motion caps.
- Check straight/turn direction and source acquisition/arrival time behavior; recorded g-unit/raw bag proof is separate from current hardware clock alignment.
- Confirm RTK position/heading gate loss stops despite healthy LIO; recovery does not silently rearm the operator stop latch.
- Check missing IMU/LiDAR, map/version/TF faults and controller loss under the same deadlines.

No automatic driving, source editing on Jetson, firmware flash, forcepush, production merge or removal of old assets is authorized. Optional camera observation requires the specific calibration/registered input actually used; no generic D455/example parameters are substituted. Real georeferenced prior registration and ground-to-road field evidence remain separate from synthetic policy results.

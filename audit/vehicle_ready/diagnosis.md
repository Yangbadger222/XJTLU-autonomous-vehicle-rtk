# Source replay diagnosis — goal remains active

Runtime9f1516f has a successful separate SDK/14-package Humble build, three actual native equation/failure/deskew probes,35 original final-wire faults,8 world-owner fault/latch cases and136 source/parsed/runtime parameter checks. This is software evidence, not physical acceptance.

The first full July15 replay incorrectly received PASS from a test that only required some paired output. It recorded57008 IMUs/2867 LiDAR packets,1866 native odom but only363 vehicle odom. That permissive verdict is superseded; no sustained interface PASS is claimed. The tightened85s run returns FAIL on sustained coverage, terminal vehicle stamp and the latched reference fault.

Captured native CDR minimizes the stop to two real odometry messages. The actual installed adapter callback reproduces it in under a second:0.399295330s,0.234684545m translation and23.481885443deg yaw. The original15deg guard correctly latches; it is not relaxed. Native body yaw rates are .9800423/.9178415rad/s, above the current .7 permission limit. The corresponding400 actual raw IMUs have median z .7725555rad/s/max absolute1.1608443. Historical bag motion is not automatically admissible under the later locked corridor profile.

Ranked causes and discriminating observations:

1. Unordered point offsets:248/2867 actual filtered packets have max-last >1us, max394855ns. The native parser uses the last selected offset as scan end. A corrected-constructor actual ROSWrapper probe synchronizes successfully but fails the exact scan-end assertion (10.020 versus10.035). Earlier probe attempts with a missing lidar type/KF constructor contract are retained as invalid fixtures, not bug evidence.
2. Header-clock/overlapping intervals:1147 actual packet minimum times precede the previous packet maximum end. Arrival/header clocks are preserved; no hardware-time equivalence is fabricated. Packet-end correction is tested independently first, with before-state versus after-scan failures distinguished.
3. Old-bag turning dynamics plus missing intermediate measurements: the captured step and raw gyro support this contribution. Publishing real measured state does not grant movement permission; EGO/final controls keep the original caps.

The parser fix uses the maximum accepted packet offset while retaining every point's original time, and rejects malformed count/array or invalid stride before indexing. No driver, factory calibration, firmware, physical limit or authority changes. Actual green parser/full replay verification is still required before completing this stage.

Jetson SSH at the hardware-note address100.79.128.21 timed out. The authorized non-Jetson100.88.131.52 remains available. No physical motion, firmware flash, production workspace edit or original-branch change has occurred. Ground-provider/EGO deployment configuration/target qualification remain separate unfinished work.

# Conditions before target and physical acceptance

Native software source 77d5c5a5ce26b52b22d5631d67ba88a71ad410b3 passes default16-node startup/exit,157source/parsed/effective checks and244portable cases. Actual13HMI/originalserialPTY cases pass at77d5c5a with their own identity. Architecture-specific ARM diagnostic qualification is in COHORT.json. None is physical driving acceptance.

MID360 already contains its IMU. Existing recorded mount, factory t_IL and original URDF attitude derive the nominal control transform; an extra external IMU calibration is not a missing prerequisite. STM command/feedback equations and41bag datasets have already supplied a recorded response audit. These existing inputs are available.

Start actual target acceptance with an actuator-disabled separate install and DDS domain. Read source/install identity, launch/YAML/CLI/environment overrides, current sensor timestamp/frames and exclusive TF/cmd ownership. Reuse the approved one-way ingress: native publisherGIDs are checked, oldTF/odom/control/authority/consent excluded. Verify final target serial output while disabled. Do not edit source directly on Jetson or overwrite production install.

Autonomous motion still requires actual observed road support. Corrected native July21AM30s replay acquires281matched ground frames but gives0supported cells,0vehicle-width strips and0persisted geometry. Unknown stays unknown; synthetic depth cannot grant permission. Check the recorded D455f/live depth and CameraInfo publication, source-bound existing mounting/calibration and verified road-prior registration. No image/CameraInfo/CompressedImage topics appear in121validmetadata from the122-entry accessible catalog; this does not claim all repository assets/calibration are absent.

On site verify the current flashed STM image, physical serial path and original KEY/hand-controller/estop behavior. Label and measure actual response, braking and slip. Source equations and unlabelled quiet bag intervals cannot prove which firmware is flashed or which interval was a physical emergency stop.

RTK authority loss must stop despite healthy LIO; recovery or transport return must not auto resume. Console exit/crash and late authenticated reset/start must leave consent revoked. Current final original mock serial tails are zero; physical estop remains PENDING.

The user's olderce46418 preview remains actuator-disabled. A prior read-only targetTCP22timeout is not proof the Jetson is offline. Mac synchronization remains pending disk space. No physical drive, flash, production merge or force push occurred.

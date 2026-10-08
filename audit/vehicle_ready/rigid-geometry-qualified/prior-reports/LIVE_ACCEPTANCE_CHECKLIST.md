# Conditions before target and physical acceptance

Current software source a7dc39774c4421b40a6a0abb7d1709a81bdc72b0 passes native and diagnostic ARM entry/exit qualification,13 HMI/originalserialPTY gates,157effective-value checks and223portable checks. The target Jetson deployment and hardware acceptance remain PENDING. Recorded MID360 mounting/internal-IMU/factory transforms, STM equations and 41 bags are available and already used.

Start target acceptance with actuator-disabled shadow in an isolated install/DDS domain. Verify the actual deployed source/install identity, environment and launch/YAML/CLI overrides against the protected source/parsed values; inspect sensor topics, timestamp/frames and exclusive TF/command ownership. The approved sensor ingress is one-way, checks each native publisherGID and excludes oldTF/odom/control/authority/consent. Confirm final serial output on the target while disabled before any motion acceptance. Editing source directly on Jetson is outside this task.

Motion acceptance is held by actual road evidence: existing replay produces0 qualified vehicle-width support strips. Unknown/missing returns remain unknown; a synthetic depth/grid fixture or a diagnosis window cannot grant real permission. Qualify whichever actual recorded sensor/depth/road-asset deployment supplies support; distinguish available mounting notes from unverified live timing, coverage and registration.

On site, verify the currently flashed STM image matches the recorded source, final physical serial path and original KEY/hand-controller/estop behavior. Measure command response, stopping/braking and slip under the actual deployment. Source equations and bag estimates inform these tests; they do not independently identify a flashed image or labelled physical stop event.

RTK loss must stop even with healthy LIO; restored RTK or input transport must not automatically resume. Console backend exit/crash and authenticated late reset/start must keep permission revoked. Current mock tests show final original serial zeros; no mock/simulation result is marked as physical estop acceptance.

No vehicle movement, firmware flash, production merge or force push has been performed. All old assets and tested parameters remain. The user's actuator-disabledce46418 preview is preserved; its age/source must not be mistaken for the newly qualified runtime. The last recorded target read-onlyTCP22probe timed out, which is not proof that the Jetson is offline. Mac source/report sync remains pending available disk space.

# RM C Board Firmware Snapshot

This directory is a repository-local snapshot of the STM32 RM C Board lower-controller firmware used by the XJTLU autonomous vehicle platform.

## Scope

- It is firmware source and project material for the lower controller.
- It is not a ROS 2 package and is not built by `colcon`.
- It is kept in the monorepo so the Jetson-side serial protocol and lower-controller code can be inspected together.

## Contents

The snapshot preserves the source tree, STM32/Keil project files, editor configuration, and companion artifacts that were present when the firmware was imported.

## Maintenance Boundary

- Do not treat generated firmware artifacts as ROS build outputs.
- Do not mechanically reformat or reorganize this tree during Jetson-side documentation work.
- If firmware behavior changes, record the expected serial command contract in both this README and the relevant ROS bridge documentation.

## Safety Stop Behavior

- The host command contract remains `vcx=<linear.x>,wc=<angular.z>\n` at 115200 baud.
- KEY, PS2 `X`, PS2 `B`, and gamepad-loss stop paths are expected to keep sending zero-current CAN frames instead of stopping CAN transmission.
- PS2 `B` is a backup coast-stop path, not a replacement for the `X` software disable or the red physical e-stop until bench and vehicle validation are complete.

## Historical Note

The original snapshot was imported from the RM C Board 2025 codebase and updated around 2025-04-28 for autonomous-navigation lower-controller functions.

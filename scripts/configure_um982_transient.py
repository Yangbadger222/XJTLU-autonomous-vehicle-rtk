#!/usr/bin/env python3
"""Apply volatile UM982 single-port output profiles without saving receiver state."""

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List


SUPPORTED_BAUDS = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600}


@dataclass(frozen=True)
class Profile:
    commands: List[str]
    final_baud: int


def mixed_profile(com: str, period_s: float) -> Profile:
    return Profile(
        commands=[
            f"UNLOG {com}",
            f"GPGGA {com} {period_s:g}",
            f"GPRMC {com} {period_s:g}",
            f"GPTHS {com} {period_s:g}",
            f"UNIHEADINGA {com} {period_s:g}",
            f"OBSVMB {com} {period_s:g}",
            f"OBSVHB {com} {period_s:g}",
            f"OBSVBASEB {com} ONCHANGED",
            f"GPSEPHB {com} ONCHANGED",
            f"GLOEPHB {com} ONCHANGED",
            f"BDSEPHB {com} ONCHANGED",
            f"GALEPHB {com} ONCHANGED",
            f"QZSSEPHB {com} ONCHANGED",
        ],
        final_baud=921600,
    )


def nmea_only_profile(com: str, period_s: float, final_baud: int) -> Profile:
    return Profile(
        commands=[
            f"UNLOG {com}",
            f"GPGGA {com} {period_s:g}",
            f"GPRMC {com} {period_s:g}",
            f"GPTHS {com} {period_s:g}",
            f"UNIHEADINGA {com} {period_s:g}",
            f"CONFIG {com} {final_baud}",
        ],
        final_baud=final_baud,
    )


def validate_commands(commands: Iterable[str]) -> None:
    for command in commands:
        normalized = " ".join(command.upper().split())
        if not normalized:
            raise ValueError("empty UM982 command")
        if "SAVE" in normalized:
            raise ValueError(f"persistent command is forbidden: {command}")


def open_serial(device: str, baud: int):
    try:
        import serial
    except ImportError as error:
        raise RuntimeError("pyserial is required on the Jetson") from error
    return serial.Serial(
        port=device,
        baudrate=baud,
        timeout=0.1,
        write_timeout=1.0,
        exclusive=True,
    )


def read_for(serial_port, duration_s: float) -> bytes:
    deadline = time.monotonic() + duration_s
    output = bytearray()
    while time.monotonic() < deadline:
        waiting = serial_port.in_waiting
        output.extend(serial_port.read(max(1, min(waiting, 65536))))
    return bytes(output)


def write_commands(serial_port, commands: Iterable[str], delay_s: float = 0.05) -> None:
    commands = list(commands)
    validate_commands(commands)
    for command in commands:
        serial_port.write((command + "\r\n").encode("ascii"))
        serial_port.flush()
        time.sleep(delay_s)


def require_ascii_preflight(sample: bytes, baud: int) -> None:
    if b"$" not in sample and b"#" not in sample:
        raise RuntimeError(f"no UM982 ASCII output observed at {baud} baud")


def print_profile(initial_baud: int, profile: Profile, transition_command: str = "") -> None:
    print(f"initial_baud={initial_baud}")
    if transition_command:
        print(transition_command)
    for command in profile.commands:
        print(command)
    print(f"final_baud={profile.final_baud}")
    print("receiver_state_saved=false")


def enter_mixed(args: argparse.Namespace) -> int:
    profile = mixed_profile(args.com, args.period)
    validate_commands(profile.commands)
    if args.dry_run:
        print_profile(
            args.source_baud,
            profile,
            transition_command=f"CONFIG {args.com} {profile.final_baud}",
        )
        return 0

    with open_serial(args.device, args.source_baud) as serial_port:
        require_ascii_preflight(read_for(serial_port, args.preflight_seconds), args.source_baud)
        write_commands(serial_port, [f"CONFIG {args.com} {profile.final_baud}"])

    time.sleep(args.settle_seconds)
    with open_serial(args.device, profile.final_baud) as serial_port:
        write_commands(serial_port, profile.commands)
        sample = read_for(serial_port, args.verify_seconds)

    ascii_seen = b"$" in sample or b"#" in sample
    binary_syncs = sample.count(b"\xAA\x44\xB5")
    print(f"ascii_seen={str(ascii_seen).lower()}")
    print(f"binary_syncs={binary_syncs}")
    print(f"bytes_sampled={len(sample)}")
    print("receiver_state_saved=false")
    if not ascii_seen or binary_syncs == 0:
        print(
            "mixed precheck failed; run restore-nmea or power-cycle the receiver",
            file=sys.stderr,
        )
        return 2
    return 0


def restore_nmea(args: argparse.Namespace) -> int:
    profile = nmea_only_profile(args.com, args.period, args.target_baud)
    validate_commands(profile.commands)
    if args.dry_run:
        print_profile(args.source_baud, profile)
        return 0

    with open_serial(args.device, args.source_baud) as serial_port:
        write_commands(serial_port, profile.commands)

    time.sleep(args.settle_seconds)
    with open_serial(args.device, profile.final_baud) as serial_port:
        sample = read_for(serial_port, args.verify_seconds)
    require_ascii_preflight(sample, profile.final_baud)
    print("ascii_seen=true")
    print(f"bytes_sampled={len(sample)}")
    print("receiver_state_saved=false")
    return 0


def baud(value: str) -> int:
    parsed = int(value)
    if parsed not in SUPPORTED_BAUDS:
        raise argparse.ArgumentTypeError(f"unsupported UM982 baud: {parsed}")
    return parsed


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def add_common_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--device", default="/dev/rtk_um982")
    parser.add_argument("--com", choices=("COM1", "COM2", "COM3"), default="COM1")
    parser.add_argument("--period", type=positive_float, default=0.1)
    parser.add_argument("--settle-seconds", type=positive_float, default=0.75)
    parser.add_argument("--verify-seconds", type=positive_float, default=3.0)
    parser.add_argument("--dry-run", action="store_true")


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="action", required=True)

    mixed = subparsers.add_parser("mixed", help="enter volatile 921600 mixed output")
    add_common_arguments(mixed)
    mixed.add_argument("--source-baud", type=baud, default=115200)
    mixed.add_argument("--preflight-seconds", type=positive_float, default=2.0)
    mixed.set_defaults(run=enter_mixed)

    restore = subparsers.add_parser("restore-nmea", help="restore volatile NMEA-only output")
    add_common_arguments(restore)
    restore.add_argument("--source-baud", type=baud, default=921600)
    restore.add_argument("--target-baud", type=baud, default=115200)
    restore.set_defaults(run=restore_nmea)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        return args.run(args)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

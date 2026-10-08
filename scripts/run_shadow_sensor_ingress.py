#!/usr/bin/env python3
"""Supervise a finite sensor-only native ingress attempt using stdlib only.

This entry reserves a fresh receipt before loading ROS shared libraries, so
pre-main rcl failures cannot leave a previous success as this attempt's result.
It grants no motion, source-domain publishing, identity or acceptance authority.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


def stop_child(child):
    if child is None or child.poll() is not None:
        return
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        if child.poll() is not None:
            break
        try:
            os.killpg(child.pid, signum)
        except ProcessLookupError:
            break
        try:
            child.wait(timeout=2)
        except subprocess.TimeoutExpired:
            continue


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--source-domain", type=int, required=True)
    parser.add_argument("--research-domain", type=int, required=True)
    parser.add_argument("--duration-s", type=float, required=True)
    parser.add_argument("--discovery-delay-s", type=float, default=0.)
    parser.add_argument("--audit-payloads", choices=("true", "false"), default="false")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        parser.error("localhost-only transport required")
    if not (0 <= args.source_domain <= 232 and 0 <= args.research_domain <= 232
            and args.source_domain != args.research_domain):
        parser.error("two distinct explicit ROS domains in 0..232 required")
    if (not math.isfinite(args.duration_s) or not 0 < args.duration_s <= 3600
            or not math.isfinite(args.discovery_delay_s)
            or not 0 <= args.discovery_delay_s <= 5
            or args.discovery_delay_s >= args.duration_s):
        parser.error("finite duration and bounded discovery delay required")
    if not args.output.is_absolute() or not args.executable.is_absolute():
        parser.error("absolute executable and receipt paths required")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    attempt_id = str(uuid.uuid4())
    started = time.monotonic()
    initial = {"status": "INCOMPLETE", "attempt_id": attempt_id,
               "termination": "not_finalized", "scope": "Not an acceptance or qualification pass."}
    try:
        with args.output.open("x") as stream:
            stream.write(json.dumps(initial) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
    except FileExistsError:
        parser.error("receipt already exists; preserve the previous attempt and use a new path")

    child, native, error, exit_code = None, None, None, None
    stop_signal, timed_out = None, False

    def on_stop(signum, _frame):
        nonlocal stop_signal
        stop_signal = signum
        if child is not None and child.poll() is None:
            try:
                os.killpg(child.pid, signum)
            except ProcessLookupError:
                pass

    previous_handlers = {signum: signal.signal(signum, on_stop)
                         for signum in (signal.SIGINT, signal.SIGTERM)}
    status, termination = "FAILED", "exception"
    attempt_dir = args.output.parent / ("shadow-ingress-" + attempt_id)
    command = []
    try:
        attempt_dir.mkdir()
        native_path = attempt_dir / "native.json"
        command = [str(args.executable), "--source-domain", str(args.source_domain),
                   "--research-domain", str(args.research_domain),
                   "--duration-s", str(args.duration_s),
                   "--discovery-delay-s", str(args.discovery_delay_s),
                   "--audit-payloads", args.audit_payloads, "--output", str(native_path)]
        with (attempt_dir / "stdout.log").open("wb") as stdout, (attempt_dir / "stderr.log").open("wb") as stderr:
            if stop_signal is None:
                child = subprocess.Popen(command, stdout=stdout, stderr=stderr, start_new_session=True)
                try:
                    child.wait(timeout=args.duration_s + 10)
                except subprocess.TimeoutExpired:
                    timed_out = True
                    stop_child(child)
                exit_code = child.returncode
        if native_path.exists():
            native = json.loads(native_path.read_text())
        if timed_out:
            status, termination, error = "FAILED", "wall_timeout", "native ingress exceeded finite wall budget"
        elif stop_signal is not None:
            status, termination = "INTERRUPTED", signal.Signals(stop_signal).name
        elif exit_code == 0 and native and native.get("status") == "COMPLETED":
            status, termination = "COMPLETED", "duration"
        else:
            status, termination = "FAILED", "native_exit"
            error = "native process did not complete with a matching finalized receipt"
    except Exception as failure:
        status, termination, error = "FAILED", "exception", f"{type(failure).__name__}: {failure}"
    finally:
        stop_child(child)
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)

        def log_tail(name):
            path = attempt_dir / name
            if not path.exists():
                return ""
            with path.open("rb") as stream:
                stream.seek(max(0, path.stat().st_size - 8192))
                return stream.read().decode("utf-8", errors="replace")

        result = {
            "status": status, "attempt_id": attempt_id, "termination": termination,
            "error": error, "native_exit": exit_code, "timed_out": timed_out,
            "source_domain": args.source_domain, "research_domain": args.research_domain,
            "wall_s": time.monotonic() - started, "command": command,
            "native_receipt": native, "native_stderr_tail": log_tail("stderr.log"),
            "native_stdout_tail": log_tail("stdout.log"),
            "executable_sha256": hashlib.sha256(args.executable.read_bytes()).hexdigest()
                if args.executable.is_file() else None,
            "launcher_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "scope": "Supervised finite one-way native sensor ingress. A fresh attempt receipt is reserved before loading ROS libraries. Completion describes only process termination; sensor identity, live timing, security, target shadow, localization, road evidence and physical acceptance remain unqualified.",
        }
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result))
    if status == "COMPLETED":
        return 0
    if status == "INTERRUPTED":
        return 128 + stop_signal
    return 1


if __name__ == "__main__":
    sys.exit(main())

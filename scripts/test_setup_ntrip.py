#!/usr/bin/env python3
"""Regression tests for runtime-only UM982 NTRIP profile management."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import stat
import sys
import tempfile
import unittest

import yaml


SCRIPT_PATH = Path(__file__).with_name("setup_ntrip.py")
SPEC = importlib.util.spec_from_file_location("setup_ntrip", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
SETUP_NTRIP = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SETUP_NTRIP
SPEC.loader.exec_module(SETUP_NTRIP)


class SetupNtripTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.config_dir = Path(self.temp_dir.name)
        self._write_profile("um982_cors.yaml", "/um982_rtk_driver", 115200)
        self._write_profile("um982_cors_mixed.yaml", "um982_rtk_driver", 921600)

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def _write_profile(self, filename: str, root: str, baud: int) -> None:
        content = {
            root: {
                "ros__parameters": {
                    "port": "/dev/rtk_um982",
                    "baud": baud,
                    "ntrip": {"enabled": False},
                }
            }
        }
        (self.config_dir / filename).write_text(
            yaml.safe_dump(content, sort_keys=False), encoding="utf-8"
        )

    def test_profile_aliases_match_transport_profiles(self) -> None:
        self.assertEqual(SETUP_NTRIP.resolve_profile("old").key, "standard")
        self.assertEqual(SETUP_NTRIP.resolve_profile("new").key, "mixed")

    def test_setup_writes_both_profiles_and_selects_active_yaml(self) -> None:
        stamp = "test"
        for profile in SETUP_NTRIP.PROFILES.values():
            SETUP_NTRIP.write_profile(
                self.config_dir,
                profile,
                "115.120.84.88",
                8103,
                "RTCM33GRCEJ",
                "account",
                stamp,
            )
        SETUP_NTRIP.write_environment(
            self.config_dir, SETUP_NTRIP.PROFILES["mixed"], "secret", stamp
        )

        for profile in SETUP_NTRIP.PROFILES.values():
            _, _, ntrip = SETUP_NTRIP.load_profile(self.config_dir, profile)
            self.assertTrue(ntrip["enabled"])
            self.assertEqual(ntrip["host"], "115.120.84.88")
            self.assertEqual(ntrip["port"], 8103)
            self.assertEqual(ntrip["mountpoint"], "RTCM33GRCEJ")
            self.assertEqual(ntrip["password"], "")
            self.assertEqual(ntrip["password_env"], "NTRIP_PASSWORD")

        env = SETUP_NTRIP.env_file(self.config_dir)
        self.assertEqual(SETUP_NTRIP.active_profile(self.config_dir).key, "mixed")
        self.assertEqual(
            SETUP_NTRIP.read_shell_export(env, "FYP_RTK_PARAMS_FILE"),
            str(self.config_dir / "um982_cors_mixed.yaml"),
        )
        self.assertEqual(SETUP_NTRIP.read_shell_export(env, "NTRIP_PASSWORD"), "secret")
        self.assertEqual(stat.S_IMODE(env.stat().st_mode), 0o600)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env bash
set -eo pipefail
timeout --signal=INT --kill-after=20s 560s bash /research-ws/adapter-signal-qa/phase-body.sh

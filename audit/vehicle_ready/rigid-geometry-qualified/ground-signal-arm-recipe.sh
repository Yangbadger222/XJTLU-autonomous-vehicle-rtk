#!/usr/bin/env bash
set -eo pipefail
timeout --signal=INT --kill-after=20s 560s bash /research-ws/ground-signal-qa/phase-body.sh

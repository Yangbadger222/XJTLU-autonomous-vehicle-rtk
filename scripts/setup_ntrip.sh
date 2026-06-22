#!/bin/bash

# Run this script with source! i.e. source scripts/setup_ntrip.sh

# Get the directory where the script is located
if [ -n "$BASH_SOURCE" ]; then
    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
else
    DIR="$( cd "$( dirname "$0" )" && pwd )"
fi

# Run the python script to parse and test
python3 "$DIR/setup_ntrip.py"
RET=$?

if [ $RET -eq 0 ] && [ -f /tmp/ntrip_env.sh ]; then
    source /tmp/ntrip_env.sh
    echo "Done! The environment variables NTRIP_PASSWORD and FYP_RTK_PARAMS_FILE are set."
    echo "Hint: If you open a new terminal, you don't need to paste the password again."
    echo "      Just run: source /tmp/ntrip_env.sh"
else
    echo "Setup aborted or failed."
fi

#!/usr/bin/env bash
# Restart all 5 SITL tmux sessions WITHOUT wiping EEPROM.
# Use this after setting CHOREO_ENABLE=1 etc, so AP_Choreo::init() re-runs
# and actually loads /APM/choreo.csv.
set -euo pipefail
cd /home/zyh/ardupilot/sitl_test
bash stop_5_sitl.sh
sleep 2
# DO NOT wipe eeprom -- we want CHOREO_ENABLE=1 to persist
bash start_5_sitl.sh

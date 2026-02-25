#!/usr/bin/env bash

MODEL="$(tr -d '\0' < "/sys/firmware/devicetree/base/model")"
export MODEL

if [ "$MODEL" = "comma tici" ]; then
  # Force a failure if the launcher doesn't exist
  [ -x "$C3_LAUNCH_SH" ] || false

  # If it exists, run it
  exec "$C3_LAUNCH_SH"
fi

exec ./launch_chffrplus.sh

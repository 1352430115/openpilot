#!/usr/bin/env python3

from openpilot.common.params import Params

UNREGISTERED_DONGLE_ID = "LOCAL_DEVICE"

def is_registered_device() -> bool:
  return True

def register(show_spinner=False) -> str | None:
  params = Params()

  dongle_id = "LOCAL_DEVICE"

  params.put("DongleId", dongle_id)

  print(f"[LOCAL MODE] DongleId: {dongle_id}")

  return dongle_id


if __name__ == "__main__":
  print(register())
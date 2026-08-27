"""Default configuration for the wired STM32 RTK test."""

from __future__ import annotations

import os


RTCM_PREAMBLE = 0xD3

DEFAULT_NTRIP_HOST = os.getenv("NTRIP_HOST", "www.gnssdata.or.kr")
DEFAULT_NTRIP_PORT = int(os.getenv("NTRIP_PORT", "2101"))
DEFAULT_NTRIP_MOUNTPOINT = os.getenv("NTRIP_MOUNTPOINT", "SUWN-RTCM31")
DEFAULT_NTRIP_USER = os.getenv("NTRIP_USER", "")
DEFAULT_NTRIP_PASSWORD = os.getenv("NTRIP_PASSWORD", "")

DEFAULT_SERIAL_PORT = os.getenv("RTK_SERIAL_PORT", "COM8")
DEFAULT_MONITOR_PORT = os.getenv("RTK_MONITOR_PORT", "COM7")
DEFAULT_SERIAL_BAUD = int(os.getenv("RTK_SERIAL_BAUD", "115200"))

# One common gate is shared by RTCM 1004/1006/1012/1230.  Frames arriving
# inside the short epoch window pass immediately; no frame is stored for later.
RTCM_FORWARD_INTERVAL = float(os.getenv("RTCM_FORWARD_INTERVAL", "2.0"))
RTCM_EPOCH_WINDOW = float(os.getenv("RTCM_EPOCH_WINDOW", "0.35"))

NTRIP_CONNECT_TIMEOUT = 10.0
NTRIP_READ_TIMEOUT = 1.0
NTRIP_RECONNECT_DELAY = 3.0
NTRIP_CHUNK_SIZE = 4096

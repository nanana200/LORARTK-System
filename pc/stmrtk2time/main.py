"""NTRIP -> TTGO RTCM transport with a separate STM32 ST-LINK monitor."""

from __future__ import annotations

import argparse
import re
import socket
import sys
import threading
import time
from pathlib import Path
from typing import BinaryIO

import serial
from serial.tools import list_ports

import config
from ntrip_client import (
    NtripAuthenticationError,
    NtripClient,
    NtripConnectionLost,
    NtripError,
    NtripMountpointError,
    NtripSettings,
)
from rtcm3 import RTCM_PASS_TYPES, Rtcm3TimeGateFilter
from serial_bridge import SerialBridge


class Console:
    def __init__(self) -> None:
        self._lock = threading.Lock()

    def print(self, message: str = "") -> None:
        with self._lock:
            print(message, flush=True)


class Stm32LogMonitor:
    _LAT_RE = re.compile(r"\[GNSS\]\s+LAT\s*=\s*(\S+)")
    _LON_RE = re.compile(r"\[GNSS\]\s+LON\s*=\s*(\S+)")

    def __init__(self, console: Console) -> None:
        self.console = console
        self.latitude = "unknown"
        self.longitude = "unknown"
        self._fix_announced = False

    def handle_line(self, line: str) -> None:
        if not line:
            return
        self.console.print(f"[STM32] {line}")
        self._process_gga(line)

        latitude_match = self._LAT_RE.search(line)
        if latitude_match:
            self.latitude = latitude_match.group(1)
        longitude_match = self._LON_RE.search(line)
        if longitude_match:
            self.longitude = longitude_match.group(1)

        normalized = line.upper()
        if "[RTK" in normalized and "FIX" in normalized and "FLOAT" not in normalized:
            if not self._fix_announced:
                self.console.print("==============================")
                self.console.print("          RTK FIX")
                self.console.print("==============================")
                self.console.print(f"LAT : {self.latitude}")
                self.console.print(f"LON : {self.longitude}")
                self.console.print("==============================")
                self._fix_announced = True
        elif "[RTK" in normalized and ("FLOAT" in normalized or "SINGLE" in normalized):
            self._fix_announced = False

    def _process_gga(self, line: str) -> None:
        starts = [index for index in (line.find("$GNGGA,"), line.find("$GPGGA,")) if index >= 0]
        if not starts:
            return
        sentence = line[min(starts):].split("*", 1)[0]
        fields = sentence.split(",")
        if len(fields) <= 13:
            return
        try:
            latitude = self._coordinate(fields[2], fields[3], 2)
            longitude = self._coordinate(fields[4], fields[5], 3)
        except (ValueError, IndexError):
            return

        quality = fields[6] or "0"
        status = {
            "0": "INVALID",
            "1": "GNSS FIX",
            "2": "DGPS / DGNSS",
            "4": "RTK FIXED",
            "5": "RTK FLOAT",
            "6": "ESTIMATED",
        }.get(quality, "UNKNOWN")
        satellites = fields[7] or "unknown"
        hdop = fields[8] or "unknown"
        differential_age = fields[13] or "N/A"
        self.latitude = f"{latitude:.8f}"
        self.longitude = f"{longitude:.8f}"
        self.console.print(
            f"[GGA] LAT={self.latitude}, LON={self.longitude}, "
            f"quality={quality} ({status}), satellites={satellites}, "
            f"HDOP={hdop}, differential_age={differential_age}"
        )

    @staticmethod
    def _coordinate(value: str, hemisphere: str, degree_digits: int) -> float:
        degrees = float(value[:degree_digits])
        minutes = float(value[degree_digits:])
        coordinate = degrees + minutes / 60.0
        if hemisphere in ("S", "W"):
            coordinate = -coordinate
        elif hemisphere not in ("N", "E"):
            raise ValueError("invalid hemisphere")
        return coordinate


class TrafficStats:
    def __init__(self, rtcm_filter: Rtcm3TimeGateFilter) -> None:
        self.rtcm_filter = rtcm_filter
        self.interval_tx = 0
        self.total_tx = 0
        self.last_report = time.monotonic()

    def add_transmitted(self, transmitted: int) -> None:
        self.interval_tx += transmitted
        self.total_tx += transmitted

    def report_if_due(self, console: Console, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_report < 1.0:
            return
        delta = self.rtcm_filter.take_interval()
        console.print(f"[NTRIP] RAW RTCM = {delta.raw_bytes} B/s")
        console.print(f"[FILTER] TYPE DROP = {delta.type_drop_bytes} B/s")
        console.print(f"[FILTER] 2SEC DROP = {delta.interval_drop_bytes} B/s")
        console.print(f"[FILTER] PASS = {delta.forwarded_bytes} B/s")
        console.print(f"[PC->TTGO] TX = {self.interval_tx} B/s")
        console.print(
            f"[RTCM] CRC errors={delta.crc_errors}, "
            f"pending={self.rtcm_filter.pending_bytes} B"
        )
        if delta.pass_epochs:
            console.print("[RTCM 2SEC]")
            console.print(f"Epoch = PASS x{delta.pass_epochs}")
            console.print(f"Interval = {self.rtcm_filter.forward_interval:.1f} sec")
        if delta.pass_types:
            passed = "  ".join(
                f"{message_type} x{count}"
                for message_type, count in sorted(delta.pass_types.items())
            )
            console.print(f"[PASS] {passed}")
            console.print(f"PASS bytes = {delta.forwarded_bytes}")
        if delta.type_drop_types:
            dropped = "  ".join(
                f"{message_type} x{count}"
                for message_type, count in sorted(delta.type_drop_types.items())
            )
            console.print(f"[TYPE DROP] {dropped}")
        if delta.interval_drop_types:
            interval_dropped = "  ".join(
                f"{message_type} x{count}"
                for message_type, count in sorted(delta.interval_drop_types.items())
            )
            console.print("[RTCM 2SEC]")
            console.print(f"Epoch = DROP x{delta.interval_drop_epochs}")
            console.print("Reason = 2SEC_INTERVAL")
            console.print(f"[2SEC DROP] {interval_dropped}")
            console.print(f"2SEC DROP bytes = {delta.interval_drop_bytes}")

        total_raw = self.rtcm_filter.total_raw_bytes
        type_reduction = self.rtcm_filter.total_type_drop_bytes / total_raw * 100.0 if total_raw else 0.0
        additional_reduction = self.rtcm_filter.total_interval_drop_bytes / total_raw * 100.0 if total_raw else 0.0
        overall_reduction = self.rtcm_filter.total_dropped_bytes / total_raw * 100.0 if total_raw else 0.0
        console.print("[RTCM TOTAL]")
        console.print(f"Raw RTCM           = {total_raw} bytes")
        console.print(f"Type Filter Drop   = {self.rtcm_filter.total_type_drop_bytes} bytes")
        console.print(f"2sec Interval Drop = {self.rtcm_filter.total_interval_drop_bytes} bytes")
        console.print(f"CRC/Junk Drop      = {self.rtcm_filter.total_crc_drop_bytes + self.rtcm_filter.total_junk_drop_bytes} bytes")
        console.print(f"Forwarded          = {self.rtcm_filter.total_forwarded_bytes} bytes")
        console.print(f"Serial TX          = {self.total_tx} bytes")
        console.print(f"Type Reduction       = {type_reduction:.2f} %")
        console.print(f"2sec Additional Reduction = {additional_reduction:.2f} %")
        console.print(f"Overall Reduction    = {overall_reduction:.2f} %")
        console.print()
        self.interval_tx = 0
        self.last_report = now


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Forward NTRIP RTCM3 to TTGO and display STM32 RTK logs from ST-LINK."
    )
    parser.add_argument(
        "--port",
        default=config.DEFAULT_SERIAL_PORT,
        help="RTCM transmit port connected to TTGO, e.g. COM8",
    )
    parser.add_argument(
        "--monitor-port",
        default=config.DEFAULT_MONITOR_PORT,
        help="STM32 ST-LINK VCP used only for debug/GGA receive, e.g. COM7",
    )
    parser.add_argument("--baud", type=int, default=config.DEFAULT_SERIAL_BAUD)
    parser.add_argument("--list-ports", action="store_true", help="list available COM ports and exit")
    parser.add_argument(
        "--interval",
        type=float,
        default=config.RTCM_FORWARD_INTERVAL,
        help="shared RTCM correction gate interval in seconds",
    )
    parser.add_argument(
        "--epoch-window",
        type=float,
        default=config.RTCM_EPOCH_WINDOW,
        help="short pass window for frames belonging to the same epoch",
    )

    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--serial-only", action="store_true", help="only display STM32 serial output")
    mode.add_argument("--rtcm-file", type=Path, help="replay RTCM3 data from a binary file")
    parser.add_argument("--file-rate", type=float, default=1000.0, help="file replay rate in bytes/s; 0=unlimited")
    parser.add_argument("--file-loop", action="store_true", help="repeat RTCM file until Ctrl+C")

    parser.add_argument("--host", default=config.DEFAULT_NTRIP_HOST)
    parser.add_argument("--ntrip-port", type=int, default=config.DEFAULT_NTRIP_PORT)
    parser.add_argument("--mountpoint", default=config.DEFAULT_NTRIP_MOUNTPOINT)
    parser.add_argument("--user", default=config.DEFAULT_NTRIP_USER)
    parser.add_argument("--password", default=config.DEFAULT_NTRIP_PASSWORD)
    parser.add_argument("--save-rtcm", type=Path, help="optionally save received raw RTCM to this file")
    return parser


def print_ports(console: Console) -> None:
    ports = sorted(list_ports.comports(), key=lambda item: item.device)
    if not ports:
        console.print("No COM ports found.")
        return
    for port in ports:
        console.print(f"{port.device} - {port.description}")


def print_banner(args: argparse.Namespace, console: Console) -> None:
    console.print("========================================")
    console.print(" RTK PC 2-SECOND TEST PROGRAM")
    console.print("========================================")
    console.print()
    if args.serial_only:
        console.print("MODE              : Serial receive only")
    elif args.rtcm_file:
        console.print(f"RTCM File         : {args.rtcm_file}")
    else:
        console.print(f"NTRIP Host       : {args.host}:{args.ntrip_port}")
        console.print(f"NTRIP Mountpoint : {args.mountpoint}")
    if not args.serial_only:
        console.print(f"RTCM TX Port     : {args.port} (TTGO)")
    console.print(f"STM Monitor Port : {args.monitor_port} (ST-LINK)")
    console.print(f"Baudrate         : {args.baud}")
    console.print(f"RTCM PASS Types  : {', '.join(str(value) for value in sorted(RTCM_PASS_TYPES))}")
    console.print(f"Forward Interval : {args.interval:.2f} sec")
    console.print(f"Epoch Window     : {args.epoch_window:.2f} sec")
    console.print()


def wait_interruptibly(stop_event: threading.Event, seconds: float) -> None:
    stop_event.wait(seconds)


def forward_chunk(
    chunk: bytes,
    bridge: SerialBridge,
    rtcm_filter: Rtcm3TimeGateFilter,
    stats: TrafficStats,
    capture: BinaryIO | None,
) -> None:
    # Save RAW input, then send only complete CRC-valid allowlisted frames.
    if capture is not None:
        capture.write(chunk)
        capture.flush()
    filtered = rtcm_filter.feed(chunk)
    transmitted = bridge.write(filtered) if filtered else 0
    stats.add_transmitted(transmitted)


def run_ntrip(
    args: argparse.Namespace,
    bridge: SerialBridge,
    console: Console,
    stop_event: threading.Event,
    serial_error: threading.Event,
    rtcm_filter: Rtcm3TimeGateFilter,
    stats: TrafficStats,
    capture: BinaryIO | None,
) -> None:
    settings = NtripSettings(
        host=args.host,
        port=args.ntrip_port,
        mountpoint=args.mountpoint,
        username=args.user,
        password=args.password,
        connect_timeout=config.NTRIP_CONNECT_TIMEOUT,
        read_timeout=config.NTRIP_READ_TIMEOUT,
        chunk_size=config.NTRIP_CHUNK_SIZE,
    )

    while not stop_event.is_set() and not serial_error.is_set():
        client = NtripClient(settings)
        try:
            console.print("[NTRIP] Connecting...")
            client.connect()
            console.print("[NTRIP] Connected")
            console.print("----------------------------------------")
            console.print()

            while not stop_event.is_set() and not serial_error.is_set():
                chunk = client.read()
                if chunk:
                    forward_chunk(chunk, bridge, rtcm_filter, stats, capture)
                stats.report_if_due(console)
        except NtripAuthenticationError as exc:
            console.print(f"[NTRIP] Authentication failed: {exc}")
        except NtripMountpointError as exc:
            console.print(f"[NTRIP] Mountpoint not found: {exc}")
        except (socket.timeout, TimeoutError) as exc:
            console.print(f"[NTRIP] Connection timeout: {exc}")
        except (NtripConnectionLost, NtripError, OSError) as exc:
            console.print(f"[NTRIP] Connection lost: {exc}")
        finally:
            client.close()

        if not stop_event.is_set() and not serial_error.is_set():
            console.print(
                f"[NTRIP] Reconnecting in {config.NTRIP_RECONNECT_DELAY:g} sec..."
            )
            wait_interruptibly(stop_event, config.NTRIP_RECONNECT_DELAY)


def run_file_replay(
    args: argparse.Namespace,
    bridge: SerialBridge,
    console: Console,
    stop_event: threading.Event,
    serial_error: threading.Event,
    rtcm_filter: Rtcm3TimeGateFilter,
    stats: TrafficStats,
    capture: BinaryIO | None,
) -> None:
    assert args.rtcm_file is not None
    path = args.rtcm_file.expanduser().resolve()
    console.print(f"[FILE] Replaying {path}")
    deadline = time.monotonic()

    while not stop_event.is_set() and not serial_error.is_set():
        with path.open("rb") as source:
            while not stop_event.is_set() and not serial_error.is_set():
                chunk = source.read(512)
                if not chunk:
                    break
                forward_chunk(chunk, bridge, rtcm_filter, stats, capture)
                if args.file_rate > 0:
                    deadline += len(chunk) / args.file_rate
                    delay = deadline - time.monotonic()
                    if delay > 0:
                        wait_interruptibly(stop_event, delay)
                    else:
                        deadline = time.monotonic()
                stats.report_if_due(console)
        if not args.file_loop:
            break
        console.print("[FILE] End reached; restarting")

    stats.report_if_due(console, force=True)
    console.print("[FILE] Replay complete")


def open_capture(path: Path | None) -> BinaryIO | None:
    if path is None:
        return None
    resolved = path.expanduser().resolve()
    resolved.parent.mkdir(parents=True, exist_ok=True)
    return resolved.open("wb")


def main() -> int:
    args = build_parser().parse_args()
    console = Console()
    if args.list_ports:
        print_ports(console)
        return 0
    if args.interval <= 0.0:
        console.print("[CONFIG] --interval must be greater than zero")
        return 2
    if args.epoch_window < 0.0 or args.epoch_window >= args.interval:
        console.print("[CONFIG] --epoch-window must be >= 0 and smaller than --interval")
        return 2
    if not args.serial_only and args.port.upper() == args.monitor_port.upper():
        console.print("[CONFIG] TTGO TX port and STM monitor port must be different")
        return 2

    print_banner(args, console)
    stop_event = threading.Event()
    serial_error = threading.Event()
    monitor = Stm32LogMonitor(console)

    def monitor_failed(message: str) -> None:
        console.print(f"[STM MONITOR] Connection lost: {message}")
        serial_error.set()

    def ttgo_failed(message: str) -> None:
        console.print(f"[TTGO SERIAL] Connection lost: {message}")
        serial_error.set()

    def handle_ttgo_line(line: str) -> None:
        if line:
            console.print(f"[TTGO] {line}")

    monitor_bridge = SerialBridge(
        args.monitor_port, args.baud, monitor.handle_line, monitor_failed
    )
    tx_bridge: SerialBridge | None = None
    capture: BinaryIO | None = None
    try:
        console.print(f"[STM MONITOR] Opening {args.monitor_port}...")
        monitor_bridge.open()
        console.print("[STM MONITOR] Connected")

        if not args.serial_only:
            tx_bridge = SerialBridge(args.port, args.baud, handle_ttgo_line, ttgo_failed)
            console.print(f"[TTGO SERIAL] Opening {args.port}...")
            tx_bridge.open()
            console.print("[TTGO SERIAL] Connected")

        capture = open_capture(args.save_rtcm)
        if capture is not None:
            console.print(f"[RTCM] Saving raw stream to {args.save_rtcm}")

        rtcm_filter = Rtcm3TimeGateFilter(
            forward_interval=args.interval,
            epoch_window=args.epoch_window,
        )
        stats = TrafficStats(rtcm_filter)

        if args.serial_only:
            console.print("[MODE] Serial-only; waiting for STM32 output. Press Ctrl+C to stop.")
            while not stop_event.is_set() and not serial_error.is_set():
                wait_interruptibly(stop_event, 0.2)
        elif args.rtcm_file:
            assert tx_bridge is not None
            run_file_replay(
                args, tx_bridge, console, stop_event, serial_error, rtcm_filter, stats, capture
            )
        else:
            assert tx_bridge is not None
            run_ntrip(
                args, tx_bridge, console, stop_event, serial_error, rtcm_filter, stats, capture
            )

        return 1 if serial_error.is_set() else 0
    except KeyboardInterrupt:
        console.print("Stopping...")
        stop_event.set()
        return 0
    except (OSError, serial.SerialException, serial.SerialTimeoutException) as exc:
        console.print(f"[SERIAL] Error: {exc}")
        return 2
    finally:
        stop_event.set()
        if capture is not None:
            capture.close()
        console.print("Closing NTRIP connection...")
        if tx_bridge is not None:
            console.print(f"Closing TTGO {args.port}...")
            tx_bridge.close()
        console.print(f"Closing STM monitor {args.monitor_port}...")
        monitor_bridge.close()
        console.print("Done.")


if __name__ == "__main__":
    sys.exit(main())

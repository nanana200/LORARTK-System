"""Dependency-free RTCM3 decoder, analyzer, and whole-frame filter."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from typing import Callable, Iterable, Optional, Union

import time

from config import RTCM_PREAMBLE


CRC24Q_POLY = 0x1864CFB
RTCM_PASS_TYPES = frozenset({1004, 1006, 1012, 1230})


def crc24q(data: Union[bytes, bytearray, memoryview]) -> int:
    crc = 0
    for value in data:
        crc ^= value << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= CRC24Q_POLY
    return crc & 0xFFFFFF


@dataclass(frozen=True)
class DecodedFrame:
    data: bytes
    message_type: Optional[int]
    crc_ok: bool


class Rtcm3FrameDecoder:
    """Reassembles RTCM3 frames across arbitrary socket.recv() boundaries."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.discarded_junk_bytes = 0

    @property
    def pending_bytes(self) -> int:
        return len(self._buffer)

    def feed(self, chunk: bytes) -> list[DecodedFrame]:
        if chunk:
            self._buffer.extend(chunk)
        frames: list[DecodedFrame] = []

        while True:
            preamble_index = self._buffer.find(RTCM_PREAMBLE)
            if preamble_index < 0:
                self.discarded_junk_bytes += len(self._buffer)
                self._buffer.clear()
                return frames
            if preamble_index:
                self.discarded_junk_bytes += preamble_index
                del self._buffer[:preamble_index]

            if len(self._buffer) < 3:
                return frames
            if self._buffer[1] & 0xFC:
                self.discarded_junk_bytes += 1
                del self._buffer[0]
                continue

            payload_length = ((self._buffer[1] & 0x03) << 8) | self._buffer[2]
            frame_length = 3 + payload_length + 3
            if len(self._buffer) < frame_length:
                return frames

            frame = bytes(self._buffer[:frame_length])
            del self._buffer[:frame_length]
            received_crc = int.from_bytes(frame[-3:], "big")
            crc_ok = crc24q(frame[:-3]) == received_crc
            message_type = None
            if payload_length >= 2:
                message_type = (frame[3] << 4) | (frame[4] >> 4)
            frames.append(DecodedFrame(frame, message_type, crc_ok))


@dataclass(frozen=True)
class AnalyzerDelta:
    valid_frames: int
    crc_errors: int
    message_types: Counter[int]


class Rtcm3Analyzer:
    """Compatibility analyzer retained from the working stmrtk project."""

    def __init__(self) -> None:
        self._decoder = Rtcm3FrameDecoder()
        self.total_valid_frames = 0
        self.total_crc_errors = 0
        self.total_preambles = 0
        self._interval_valid_frames = 0
        self._interval_crc_errors = 0
        self._interval_types: Counter[int] = Counter()

    def feed(self, chunk: bytes) -> None:
        self.total_preambles += chunk.count(RTCM_PREAMBLE)
        for frame in self._decoder.feed(chunk):
            if not frame.crc_ok:
                self.total_crc_errors += 1
                self._interval_crc_errors += 1
                continue
            self.total_valid_frames += 1
            self._interval_valid_frames += 1
            if frame.message_type is not None:
                self._interval_types[frame.message_type] += 1

    def take_interval(self) -> AnalyzerDelta:
        delta = AnalyzerDelta(
            valid_frames=self._interval_valid_frames,
            crc_errors=self._interval_crc_errors,
            message_types=self._interval_types.copy(),
        )
        self._interval_valid_frames = 0
        self._interval_crc_errors = 0
        self._interval_types.clear()
        return delta


@dataclass(frozen=True)
class FilterDelta:
    raw_bytes: int
    forwarded_bytes: int
    dropped_bytes: int
    passed_frames: int
    dropped_frames: int
    crc_errors: int
    pass_types: Counter[int]
    drop_types: Counter[int]


class Rtcm3FrameFilter:
    """Passes complete, CRC-valid frames whose type is explicitly allowed."""

    def __init__(self, pass_types: Iterable[int] = RTCM_PASS_TYPES) -> None:
        self.pass_types = frozenset(pass_types)
        self._decoder = Rtcm3FrameDecoder()
        self.total_raw_bytes = 0
        self.total_forwarded_bytes = 0
        self.total_dropped_bytes = 0
        self.total_passed_frames = 0
        self.total_dropped_frames = 0
        self.total_crc_errors = 0
        self._interval_raw_bytes = 0
        self._interval_forwarded_bytes = 0
        self._interval_dropped_bytes = 0
        self._interval_passed_frames = 0
        self._interval_dropped_frames = 0
        self._interval_crc_errors = 0
        self._interval_pass_types: Counter[int] = Counter()
        self._interval_drop_types: Counter[int] = Counter()

    @property
    def pending_bytes(self) -> int:
        return self._decoder.pending_bytes

    def feed(self, chunk: bytes) -> bytes:
        self.total_raw_bytes += len(chunk)
        self._interval_raw_bytes += len(chunk)
        junk_before = self._decoder.discarded_junk_bytes
        frames = self._decoder.feed(chunk)
        junk = self._decoder.discarded_junk_bytes - junk_before
        if junk:
            self._add_drop_bytes(junk)

        forwarded = bytearray()
        for frame in frames:
            frame_length = len(frame.data)
            if not frame.crc_ok:
                self.total_crc_errors += 1
                self._interval_crc_errors += 1
                self.total_dropped_frames += 1
                self._interval_dropped_frames += 1
                self._add_drop_bytes(frame_length)
                continue

            if frame.message_type in self.pass_types:
                forwarded.extend(frame.data)
                self.total_forwarded_bytes += frame_length
                self._interval_forwarded_bytes += frame_length
                self.total_passed_frames += 1
                self._interval_passed_frames += 1
                assert frame.message_type is not None
                self._interval_pass_types[frame.message_type] += 1
            else:
                self.total_dropped_frames += 1
                self._interval_dropped_frames += 1
                self._add_drop_bytes(frame_length)
                if frame.message_type is not None:
                    self._interval_drop_types[frame.message_type] += 1

        return bytes(forwarded)

    def _add_drop_bytes(self, count: int) -> None:
        self.total_dropped_bytes += count
        self._interval_dropped_bytes += count

    def take_interval(self) -> FilterDelta:
        delta = FilterDelta(
            raw_bytes=self._interval_raw_bytes,
            forwarded_bytes=self._interval_forwarded_bytes,
            dropped_bytes=self._interval_dropped_bytes,
            passed_frames=self._interval_passed_frames,
            dropped_frames=self._interval_dropped_frames,
            crc_errors=self._interval_crc_errors,
            pass_types=self._interval_pass_types.copy(),
            drop_types=self._interval_drop_types.copy(),
        )
        self._interval_raw_bytes = 0
        self._interval_forwarded_bytes = 0
        self._interval_dropped_bytes = 0
        self._interval_passed_frames = 0
        self._interval_dropped_frames = 0
        self._interval_crc_errors = 0
        self._interval_pass_types.clear()
        self._interval_drop_types.clear()
        return delta


@dataclass(frozen=True)
class TimeGateDelta:
    raw_bytes: int
    forwarded_bytes: int
    type_drop_bytes: int
    interval_drop_bytes: int
    crc_drop_bytes: int
    junk_drop_bytes: int
    pass_epochs: int
    interval_drop_epochs: int
    crc_errors: int
    pass_types: Counter[int]
    type_drop_types: Counter[int]
    interval_drop_types: Counter[int]


class Rtcm3TimeGateFilter:
    """Whole-frame type filter with one shared monotonic correction-epoch gate.

    The first target frame after ``forward_interval`` opens a short epoch
    window. Target frames in that window are returned immediately. Target
    frames outside it are discarded immediately and are never queued.
    """

    def __init__(
        self,
        pass_types: Iterable[int] = RTCM_PASS_TYPES,
        forward_interval: float = 2.0,
        epoch_window: float = 0.35,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if forward_interval <= 0.0:
            raise ValueError("forward_interval must be greater than zero")
        if epoch_window < 0.0 or epoch_window >= forward_interval:
            raise ValueError("epoch_window must be >= 0 and < forward_interval")
        self.pass_types = frozenset(pass_types)
        self.forward_interval = forward_interval
        self.epoch_window = epoch_window
        self._clock = clock
        self._decoder = Rtcm3FrameDecoder()
        self._next_gate_open_at = float("-inf")
        self._pass_window_until = float("-inf")
        self._drop_group_until = float("-inf")

        self.total_raw_bytes = 0
        self.total_forwarded_bytes = 0
        self.total_type_drop_bytes = 0
        self.total_interval_drop_bytes = 0
        self.total_crc_drop_bytes = 0
        self.total_junk_drop_bytes = 0
        self.total_pass_epochs = 0
        self.total_interval_drop_epochs = 0
        self.total_crc_errors = 0
        self._reset_interval()

    @property
    def pending_bytes(self) -> int:
        return self._decoder.pending_bytes

    @property
    def total_dropped_bytes(self) -> int:
        return (
            self.total_type_drop_bytes
            + self.total_interval_drop_bytes
            + self.total_crc_drop_bytes
            + self.total_junk_drop_bytes
        )

    def feed(self, chunk: bytes) -> bytes:
        now = self._clock()
        self.total_raw_bytes += len(chunk)
        self._interval_raw_bytes += len(chunk)
        junk_before = self._decoder.discarded_junk_bytes
        frames = self._decoder.feed(chunk)
        junk = self._decoder.discarded_junk_bytes - junk_before
        if junk:
            self.total_junk_drop_bytes += junk
            self._interval_junk_drop_bytes += junk

        forwarded = bytearray()
        for frame in frames:
            frame_length = len(frame.data)
            if not frame.crc_ok:
                self.total_crc_errors += 1
                self._interval_crc_errors += 1
                self.total_crc_drop_bytes += frame_length
                self._interval_crc_drop_bytes += frame_length
                continue

            message_type = frame.message_type
            if message_type not in self.pass_types:
                self.total_type_drop_bytes += frame_length
                self._interval_type_drop_bytes += frame_length
                if message_type is not None:
                    self._interval_type_drop_types[message_type] += 1
                continue

            assert message_type is not None
            if self._gate_allows(now):
                forwarded.extend(frame.data)
                self.total_forwarded_bytes += frame_length
                self._interval_forwarded_bytes += frame_length
                self._interval_pass_types[message_type] += 1
            else:
                self.total_interval_drop_bytes += frame_length
                self._interval_interval_drop_bytes += frame_length
                self._interval_interval_drop_types[message_type] += 1
                if now > self._drop_group_until:
                    self.total_interval_drop_epochs += 1
                    self._interval_interval_drop_epochs += 1
                    self._drop_group_until = now + self.epoch_window

        return bytes(forwarded)

    def _gate_allows(self, now: float) -> bool:
        if now >= self._next_gate_open_at:
            self.total_pass_epochs += 1
            self._interval_pass_epochs += 1
            self._pass_window_until = now + self.epoch_window
            self._next_gate_open_at = now + self.forward_interval
            return True
        return now <= self._pass_window_until

    def take_interval(self) -> TimeGateDelta:
        delta = TimeGateDelta(
            raw_bytes=self._interval_raw_bytes,
            forwarded_bytes=self._interval_forwarded_bytes,
            type_drop_bytes=self._interval_type_drop_bytes,
            interval_drop_bytes=self._interval_interval_drop_bytes,
            crc_drop_bytes=self._interval_crc_drop_bytes,
            junk_drop_bytes=self._interval_junk_drop_bytes,
            pass_epochs=self._interval_pass_epochs,
            interval_drop_epochs=self._interval_interval_drop_epochs,
            crc_errors=self._interval_crc_errors,
            pass_types=self._interval_pass_types.copy(),
            type_drop_types=self._interval_type_drop_types.copy(),
            interval_drop_types=self._interval_interval_drop_types.copy(),
        )
        self._reset_interval()
        return delta

    def _reset_interval(self) -> None:
        self._interval_raw_bytes = 0
        self._interval_forwarded_bytes = 0
        self._interval_type_drop_bytes = 0
        self._interval_interval_drop_bytes = 0
        self._interval_crc_drop_bytes = 0
        self._interval_junk_drop_bytes = 0
        self._interval_pass_epochs = 0
        self._interval_interval_drop_epochs = 0
        self._interval_crc_errors = 0
        self._interval_pass_types: Counter[int] = Counter()
        self._interval_type_drop_types: Counter[int] = Counter()
        self._interval_interval_drop_types: Counter[int] = Counter()

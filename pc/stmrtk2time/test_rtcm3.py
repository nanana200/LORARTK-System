import unittest

from rtcm3 import (
    RTCM_PASS_TYPES,
    Rtcm3Analyzer,
    Rtcm3FrameFilter,
    Rtcm3TimeGateFilter,
    crc24q,
)


def make_frame(message_type: int, tail: bytes = b"\x01\x02") -> bytes:
    payload = bytes((message_type >> 4, (message_type & 0x0F) << 4)) + tail
    header = bytes((0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF))
    body = header + payload
    return body + crc24q(body).to_bytes(3, "big")


class Rtcm3AnalyzerTest(unittest.TestCase):
    def test_fragmented_frame_and_type(self) -> None:
        frame = make_frame(1077)
        analyzer = Rtcm3Analyzer()
        analyzer.feed(frame[:2])
        analyzer.feed(frame[2:5])
        analyzer.feed(frame[5:])
        delta = analyzer.take_interval()
        self.assertEqual(delta.valid_frames, 1)
        self.assertEqual(delta.crc_errors, 0)
        self.assertEqual(delta.message_types[1077], 1)

    def test_crc_error(self) -> None:
        frame = bytearray(make_frame(1005))
        frame[-1] ^= 0x01
        analyzer = Rtcm3Analyzer()
        analyzer.feed(bytes(frame))
        delta = analyzer.take_interval()
        self.assertEqual(delta.valid_frames, 0)
        self.assertEqual(delta.crc_errors, 1)


class Rtcm3FrameFilterTest(unittest.TestCase):
    def test_only_allowlisted_whole_frames_are_forwarded(self) -> None:
        frame_1004 = make_frame(1004, b"\x10")
        frame_1019 = make_frame(1019, b"\x20\x21")
        frame_1230 = make_frame(1230, b"\x30")
        stream = frame_1004 + frame_1019 + frame_1230
        rtcm_filter = Rtcm3FrameFilter()

        # Boundaries intentionally cut both within and between RTCM frames.
        output = b""
        for chunk in (stream[:2], stream[2:11], stream[11:19], stream[19:]):
            output += rtcm_filter.feed(chunk)

        self.assertEqual(output, frame_1004 + frame_1230)
        delta = rtcm_filter.take_interval()
        self.assertEqual(delta.passed_frames, 2)
        self.assertEqual(delta.dropped_frames, 1)
        self.assertEqual(delta.pass_types[1004], 1)
        self.assertEqual(delta.pass_types[1230], 1)
        self.assertEqual(delta.drop_types[1019], 1)
        self.assertEqual(rtcm_filter.pending_bytes, 0)

    def test_crc_error_is_never_forwarded(self) -> None:
        corrupt = bytearray(make_frame(1012))
        corrupt[-2] ^= 0x80
        rtcm_filter = Rtcm3FrameFilter()
        self.assertEqual(rtcm_filter.feed(bytes(corrupt)), b"")
        delta = rtcm_filter.take_interval()
        self.assertEqual(delta.crc_errors, 1)
        self.assertEqual(delta.dropped_frames, 1)

    def test_allowlist_is_exact(self) -> None:
        self.assertEqual(RTCM_PASS_TYPES, frozenset({1004, 1006, 1012, 1230}))


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


class Rtcm3TimeGateFilterTest(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = FakeClock()
        self.rtcm_filter = Rtcm3TimeGateFilter(
            forward_interval=2.0,
            epoch_window=0.35,
            clock=self.clock,
        )
        self.target_frames = {
            message_type: make_frame(message_type, bytes((index,)))
            for index, message_type in enumerate(sorted(RTCM_PASS_TYPES), start=1)
        }

    def feed_epoch(self, start: float) -> bytes:
        output = b""
        for offset, message_type in enumerate(sorted(RTCM_PASS_TYPES)):
            self.clock.now = start + offset * 0.05
            output += self.rtcm_filter.feed(self.target_frames[message_type])
        return output

    def expected_epoch(self) -> bytes:
        return b"".join(self.target_frames[value] for value in sorted(RTCM_PASS_TYPES))

    def test_shared_gate_passes_and_drops_whole_epochs(self) -> None:
        first = self.feed_epoch(0.0)
        dropped = self.feed_epoch(1.0)
        third = self.feed_epoch(2.01)
        self.assertEqual(first, self.expected_epoch())
        self.assertEqual(dropped, b"")
        self.assertEqual(third, self.expected_epoch())

        delta = self.rtcm_filter.take_interval()
        self.assertEqual(delta.pass_epochs, 2)
        self.assertEqual(delta.interval_drop_epochs, 1)
        for message_type in RTCM_PASS_TYPES:
            self.assertEqual(delta.pass_types[message_type], 2)
            self.assertEqual(delta.interval_drop_types[message_type], 1)

    def test_dropped_epoch_is_never_sent_later(self) -> None:
        self.feed_epoch(0.0)
        dropped_epoch = self.feed_epoch(1.0)
        self.clock.now = 2.1
        latest = make_frame(1004, b"latest")
        sent = self.rtcm_filter.feed(latest)
        self.assertEqual(dropped_epoch, b"")
        self.assertEqual(sent, latest)

    def test_type_drop_and_interval_drop_are_separate(self) -> None:
        self.clock.now = 0.0
        self.rtcm_filter.feed(make_frame(1004))
        self.clock.now = 1.0
        self.rtcm_filter.feed(make_frame(1012))
        self.rtcm_filter.feed(make_frame(1019))
        delta = self.rtcm_filter.take_interval()
        self.assertEqual(delta.interval_drop_types[1012], 1)
        self.assertEqual(delta.type_drop_types[1019], 1)

    def test_fragmented_target_is_decided_only_when_complete(self) -> None:
        frame = make_frame(1230, b"fragmented")
        self.clock.now = 0.0
        self.assertEqual(self.rtcm_filter.feed(frame[:4]), b"")
        self.clock.now = 0.1
        self.assertEqual(self.rtcm_filter.feed(frame[4:]), frame)


if __name__ == "__main__":
    unittest.main()

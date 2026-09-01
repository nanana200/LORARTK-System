import importlib.util
import sys
import time
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("keytest.py")
SPEC = importlib.util.spec_from_file_location("keytest_round_robin", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
keytest = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = keytest
SPEC.loader.exec_module(keytest)


def make_rtcm(message_type: int, fill: int) -> bytes:
    payload = bytes((message_type >> 4, (message_type & 0x0F) << 4, fill))
    body = bytes((0xD3, 0, len(payload))) + payload
    return body + keytest._crc24q(body).to_bytes(3, "big")


class FakeSerial:
    def __init__(self) -> None:
        self.is_open = True
        self.written = bytearray()

    def write(self, data) -> int:
        chunk = bytes(data)
        self.written.extend(chunk)
        return len(chunk)


class RtcmRoundRobinTests(unittest.TestCase):
    def setUp(self) -> None:
        keytest._latest_rtcm_frames.clear()
        keytest._rtcm_round_robin_index = 0
        keytest._rtcm_generation = 0
        keytest._rtcm_replaced_frames = 0
        keytest._pending_waypoint = None
        keytest._cycle_count = 0
        keytest._link_events.clear()
        keytest._ser = None

    def test_all_allowed_types_are_selected_once_in_order(self) -> None:
        now = time.monotonic()
        for index, message_type in enumerate(keytest.RTCM_PASS_ORDER):
            keytest.publish_latest_rtcm_frame(
                message_type, make_rtcm(message_type, index), now
            )

        selected_types = []
        for _ in keytest.RTCM_PASS_ORDER:
            selected, _ = keytest._take_cycle_inputs()
            self.assertIsNotNone(selected)
            selected_types.append(selected.message_type)

        self.assertEqual(tuple(selected_types), keytest.RTCM_PASS_ORDER)
        selected, _ = keytest._take_cycle_inputs()
        self.assertIsNone(selected)

    def test_same_type_overwrites_only_its_own_slot(self) -> None:
        now = time.monotonic()
        old_1004 = make_rtcm(1004, 1)
        new_1004 = make_rtcm(1004, 2)
        frame_1006 = make_rtcm(1006, 3)
        keytest.publish_latest_rtcm_frame(1004, old_1004, now)
        keytest.publish_latest_rtcm_frame(1006, frame_1006, now)
        keytest.publish_latest_rtcm_frame(1004, new_1004, now)

        first, _ = keytest._take_cycle_inputs()
        second, _ = keytest._take_cycle_inputs()
        self.assertEqual(first.payload, new_1004)
        self.assertEqual(second.payload, frame_1006)
        self.assertEqual(keytest._rtcm_replaced_frames, 1)

    def test_disallowed_type_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            keytest.publish_latest_rtcm_frame(1020, make_rtcm(1020, 4))

    def test_stale_slot_is_discarded_and_next_fresh_type_is_selected(self) -> None:
        now = time.monotonic()
        keytest.publish_latest_rtcm_frame(
            1004,
            make_rtcm(1004, 5),
            now - keytest.RTCM_MAX_AGE_SECONDS - 1.0,
        )
        keytest.publish_latest_rtcm_frame(1006, make_rtcm(1006, 6), now)

        selected, _ = keytest._take_cycle_inputs()
        self.assertEqual(selected.message_type, 1006)
        self.assertNotIn(1004, keytest._latest_rtcm_frames)
        self.assertIn("stale pending type=1004", "\n".join(keytest.get_link_events()))

    def test_each_cycle_writes_only_one_rtcm_frame(self) -> None:
        now = time.monotonic()
        frame_1004 = make_rtcm(1004, 7)
        frame_1006 = make_rtcm(1006, 8)
        keytest.publish_latest_rtcm_frame(1004, frame_1004, now)
        keytest.publish_latest_rtcm_frame(1006, frame_1006, now)
        fake = FakeSerial()
        keytest._ser = fake

        keytest._run_downlink_cycle()
        self.assertEqual(
            bytes(fake.written),
            keytest._build_rtcm_packet(frame_1004) + keytest._build_enter_rx_packet(),
        )

        fake.written.clear()
        keytest._run_downlink_cycle()
        self.assertEqual(
            bytes(fake.written),
            keytest._build_rtcm_packet(frame_1006) + keytest._build_enter_rx_packet(),
        )


if __name__ == "__main__":
    unittest.main()

"""Mock Arduino/MCU tests: DISARMED default, ARM/DISARM/STOP, command TTL, link timeout.

All timing uses FakeClock, so boundaries are exact and no test sleeps.
"""

import dataclasses
import random
import unittest

import _bootstrap
from lidar_robot import protocol as p
from lidar_robot.clock import FakeClock
from lidar_robot.config import load_config
from lidar_robot.mock_mcu import MockMotorController

MOCK_CFG = load_config(_bootstrap.CONFIG_DIR / "robot.mock.toml")
LINK_TIMEOUT = 500
CAP = 500


class Harness:
    """Drives the MCU the way the Pi would; decodes replies."""

    def __init__(self, start_ms=0, **kw):
        self.clock = FakeClock(start_ms)
        kw.setdefault("link_timeout_ms", LINK_TIMEOUT)
        kw.setdefault("max_duty_permille", CAP)
        self.mcu = MockMotorController(self.clock, **kw)
        self.seq = 0
        self.last_valid_ms = None

    def raw(self, data):
        out = self.mcu.feed(data)
        asm = p.FrameAssembler()
        return asm.feed(out)

    def send(self, cmd, *fields, seq=None):
        self.seq = (self.seq + 1) & 0xFFFF if seq is None else seq
        replies = self.raw(p.encode(cmd, self.seq, *fields))
        self.last_valid_ms = self.clock.now_ms()
        return replies

    def one(self, cmd, *fields, **kw):
        replies = self.send(cmd, *fields, **kw)
        assert len(replies) == 1, replies
        assert isinstance(replies[0], p.Frame), replies
        return replies[0]

    def arm(self):
        return self.one("ARM")

    def drive(self, left, right, ttl=250):
        return self.one("DRV", left, right, ttl)

    def advance(self, ms):
        self.clock.advance(ms)

    @property
    def out(self):
        return self.mcu.outputs


class BootAndDisarmedTests(unittest.TestCase):
    def test_boots_disarmed_with_zero_outputs_and_no_fault(self):
        h = Harness()
        self.assertEqual(h.mcu.state, p.STATE_DISARMED)
        self.assertEqual(h.mcu.fault, p.FAULT_NONE)
        self.assertEqual(h.out, (0, 0))
        self.assertEqual(h.mcu.history, [(0, 0, 0)])

    def test_drv_rejected_while_disarmed(self):
        h = Harness()
        reply = h.one("DRV", 100, 100, 250)
        self.assertEqual(reply, p.Frame("NAK", ("1", p.REASON_NOT_ARMED)))
        self.assertEqual(h.out, (0, 0))
        self.assertEqual(len(h.mcu.history), 1)

    def test_drv_rejected_even_with_zero_values_while_disarmed(self):
        h = Harness()
        self.assertEqual(h.one("DRV", 0, 0, 250).fields[1], p.REASON_NOT_ARMED)

    def test_no_link_timeout_or_fault_while_disarmed(self):
        h = Harness()
        h.advance(1_000_000)
        h.mcu.tick()
        self.assertEqual((h.mcu.state, h.mcu.fault, h.out), (p.STATE_DISARMED, p.FAULT_NONE, (0, 0)))

    def test_safe_commands_work_while_disarmed(self):
        h = Harness()
        self.assertEqual(h.one("PING", seq=7), p.Frame("ACK", ("7", "DISARMED")))
        self.assertEqual(h.one("STOP", seq=8), p.Frame("ACK", ("8", "DISARMED")))
        self.assertEqual(h.one("DISARM", seq=9), p.Frame("ACK", ("9", "DISARMED")))

    def test_hello_reports_protocol_firmware_caps_and_state(self):
        h = Harness(caps=p.CAP_ENCODERS, fw_id="mock-0.1")
        self.assertEqual(h.one("HELLO", seq=3),
                         p.Frame("HELLO", ("3", str(p.PROTO_VERSION), "mock-0.1", "01", "DISARMED")))
        self.assertEqual(Harness().one("HELLO").fields[3], "00")

    def test_from_config_uses_config_limits_and_encoder_capability(self):
        clock = FakeClock()
        mcu = MockMotorController.from_config(MOCK_CFG, clock)
        asm = p.FrameAssembler()
        mcu.feed(p.encode("ARM", 1))
        reply = asm.feed(mcu.feed(p.encode("DRV", 2, MOCK_CFG.limits.max_duty_permille + 1, 0, 100)))
        self.assertEqual(reply[0].fields[1], p.REASON_RANGE)
        yes = dataclasses.replace(MOCK_CFG, encoders=dataclasses.replace(MOCK_CFG.encoders, present="yes"))
        hello = p.FrameAssembler().feed(
            MockMotorController.from_config(yes, FakeClock()).feed(p.encode("HELLO", 1)))
        self.assertEqual(hello[0].fields[3], "01")

    def test_constructor_validation(self):
        clock = FakeClock()
        for kw in ({"link_timeout_ms": 0}, {"link_timeout_ms": -1}, {"max_duty_permille": 0},
                   {"max_duty_permille": 1001}, {"caps": 256}, {"caps": -1},
                   {"fw_id": "bad,id"}, {"fw_id": "bad id"}, {"fw_id": ""}):
            with self.subTest(kw=kw):
                with self.assertRaises(ValueError):
                    MockMotorController(clock, **kw)


class ArmDisarmStopTests(unittest.TestCase):
    def test_arm_acks_and_does_not_move_motors(self):
        h = Harness()
        self.assertEqual(h.arm(), p.Frame("ACK", ("1", "ARMED")))
        self.assertEqual(h.out, (0, 0))

    def test_drive_after_arm_sets_outputs(self):
        h = Harness()
        h.arm()
        self.assertEqual(h.drive(100, -200), p.Frame("ACK", ("2", "ARMED")))
        self.assertEqual(h.out, (100, -200))

    def test_disarm_stops_motors_and_blocks_further_drive(self):
        h = Harness()
        h.arm()
        h.drive(300, 300)
        self.assertEqual(h.one("DISARM").fields[1], "DISARMED")
        self.assertEqual(h.out, (0, 0))
        self.assertEqual(h.drive(100, 100).fields[1], p.REASON_NOT_ARMED)
        self.assertEqual(h.out, (0, 0))

    def test_stop_zeroes_outputs_but_stays_armed(self):
        h = Harness()
        h.arm()
        h.drive(300, 300, 1000)
        self.assertEqual(h.one("STOP"), p.Frame("ACK", ("3", "ARMED")))
        self.assertEqual(h.out, (0, 0))
        self.assertEqual(h.mcu.ttl_remaining_ms, 0)
        self.assertEqual(h.drive(50, 50).cmd, "ACK")
        self.assertEqual(h.out, (50, 50))

    def test_rearming_while_moving_zeroes_outputs(self):
        h = Harness()
        h.arm()
        h.drive(300, 300)
        h.arm()
        self.assertEqual(h.out, (0, 0))

    def test_boundary_values_accepted(self):
        h = Harness()
        h.arm()
        for left, right in ((CAP, CAP), (-CAP, -CAP), (CAP, -CAP), (0, 0)):
            self.assertEqual(h.drive(left, right).cmd, "ACK")
            self.assertEqual(h.out, (left, right))


class CommandTtlTests(unittest.TestCase):
    def test_outputs_zero_exactly_at_ttl_and_not_before(self):
        h = Harness()
        h.arm()
        h.drive(200, 200, ttl=250)
        h.advance(249)
        h.mcu.tick()
        self.assertEqual(h.out, (200, 200))
        self.assertEqual(h.mcu.ttl_remaining_ms, 1)
        h.advance(1)
        h.mcu.tick()
        self.assertEqual(h.out, (0, 0))
        self.assertEqual(h.mcu.history[-1], (250, 0, 0))

    def test_ttl_expiry_keeps_mcu_armed_and_a_fresh_command_resumes(self):
        h = Harness()
        h.arm()
        h.drive(200, 200, 100)
        h.advance(100)
        h.mcu.tick()
        self.assertEqual(h.mcu.state, p.STATE_ARMED)
        self.assertEqual(h.mcu.fault, p.FAULT_NONE)
        h.drive(150, 150, 100)
        self.assertEqual(h.out, (150, 150))

    def test_expiry_is_evaluated_before_processing_new_bytes(self):
        # No explicit tick(): the next received frame must still see the stop first.
        h = Harness()
        h.arm()
        h.drive(200, 200, 250)
        h.advance(300)
        h.send("PING")
        self.assertEqual(h.out, (0, 0))
        self.assertEqual(h.mcu.history[-1][1:], (0, 0))

    def test_new_command_replaces_and_restarts_ttl(self):
        h = Harness()
        h.arm()
        h.drive(100, 100, 250)
        h.advance(200)
        h.drive(120, 120, 250)
        h.advance(249)
        h.mcu.tick()
        self.assertEqual(h.out, (120, 120))
        h.advance(1)
        h.mcu.tick()
        self.assertEqual(h.out, (0, 0))

    def test_ttl_bounds(self):
        h = Harness()
        h.arm()
        self.assertEqual(h.drive(10, 10, p.MAX_TTL_MS).cmd, "ACK")
        for bad in (p.MAX_TTL_MS + 1, 0, -1, 99999):
            with self.subTest(ttl=bad):
                self.assertEqual(h.drive(10, 10, bad).fields[1], p.REASON_TTL)
                self.assertEqual(h.out, (0, 0))  # fail-safe: rejected DRV stops the robot
                h.drive(10, 10, 100)

    def test_stat_reports_ttl_remaining(self):
        h = Harness()
        h.arm()
        h.advance(100)
        h.drive(100, -200, 250)
        h.advance(50)
        self.assertEqual(h.one("STAT", seq=11),
                         p.Frame("STAT", ("11", "ARMED", "100", "-200", "200", "NONE", "150")))


class LinkTimeoutTests(unittest.TestCase):
    def test_disarms_exactly_at_link_timeout_with_latched_fault(self):
        h = Harness()
        h.arm()
        h.advance(LINK_TIMEOUT - 1)
        h.mcu.tick()
        self.assertEqual(h.mcu.state, p.STATE_ARMED)
        h.advance(1)
        h.mcu.tick()
        self.assertEqual((h.mcu.state, h.mcu.fault), (p.STATE_DISARMED, p.FAULT_LINK_TIMEOUT))

    def test_any_valid_frame_refreshes_the_link_timer(self):
        h = Harness()
        h.arm()
        for _ in range(10):
            h.advance(LINK_TIMEOUT - 1)
            h.send("PING")
        self.assertEqual(h.mcu.state, p.STATE_ARMED)
        h.advance(LINK_TIMEOUT)
        h.mcu.tick()
        self.assertEqual(h.mcu.state, p.STATE_DISARMED)

    def test_link_timeout_beats_a_longer_command_ttl_and_stops_motors(self):
        h = Harness()
        h.arm()
        h.drive(300, 300, ttl=1000)
        h.advance(LINK_TIMEOUT)
        h.mcu.tick()
        self.assertEqual(h.out, (0, 0))
        self.assertEqual((h.mcu.state, h.mcu.fault), (p.STATE_DISARMED, p.FAULT_LINK_TIMEOUT))
        self.assertEqual(h.mcu.history[-1], (LINK_TIMEOUT, 0, 0))

    def test_drive_is_refused_after_link_timeout_until_rearmed(self):
        h = Harness()
        h.arm()
        h.advance(LINK_TIMEOUT)
        h.mcu.tick()
        self.assertEqual(h.drive(100, 100).fields[1], p.REASON_NOT_ARMED)
        self.assertEqual(h.one("STAT").fields[1:], ("DISARMED", "0", "0", "0", "LINK_TIMEOUT", "500"))
        h.arm()
        self.assertEqual(h.mcu.fault, p.FAULT_NONE)
        self.assertEqual(h.drive(100, 100).cmd, "ACK")

    def test_late_frame_cannot_rescue_an_expired_watchdog(self):
        h = Harness()
        h.arm()
        h.advance(LINK_TIMEOUT + 100)       # silence, and NO tick() call
        reply = h.one("DRV", 300, 300, 250)  # arrives late
        self.assertEqual(reply.fields[1], p.REASON_NOT_ARMED)
        self.assertEqual((h.mcu.state, h.mcu.fault, h.out), (p.STATE_DISARMED, p.FAULT_LINK_TIMEOUT, (0, 0)))

    def test_late_ping_does_not_rearm(self):
        h = Harness()
        h.arm()
        h.advance(LINK_TIMEOUT)
        self.assertEqual(h.one("PING").fields[1], "DISARMED")

    def test_corrupt_frames_and_garbage_do_not_refresh_the_watchdog(self):
        h = Harness()
        h.arm()
        h.advance(LINK_TIMEOUT - 100)
        good = p.encode("PING", 9)
        corrupt = good[:-3] + (b"00" if good[-3:-1] != b"00" else b"01") + b"\n"
        replies = h.raw(corrupt)
        self.assertEqual(replies, [p.Frame("NAK", ("0", p.REASON_BAD_FRAME))])
        h.raw(b"\x00\xff garbage with no start byte\n")
        h.raw(b"$PING,9")                      # unterminated partial frame
        h.advance(100)
        h.mcu.tick()
        self.assertEqual((h.mcu.state, h.mcu.fault), (p.STATE_DISARMED, p.FAULT_LINK_TIMEOUT))

    def test_rejected_but_valid_frames_still_count_as_link_activity(self):
        h = Harness()
        h.arm()
        h.advance(LINK_TIMEOUT - 1)
        self.assertEqual(h.one("DRV", 99999, 0, 100).fields[1], p.REASON_RANGE)
        h.advance(LINK_TIMEOUT - 1)
        h.mcu.tick()
        self.assertEqual(h.mcu.state, p.STATE_ARMED)


class FailSafeAndValidationTests(unittest.TestCase):
    def moving(self):
        h = Harness()
        h.arm()
        h.drive(300, 300, 1000)
        return h

    def test_range_violation_is_nakd_and_stops_a_moving_robot(self):
        for left, right in ((CAP + 1, 0), (0, -CAP - 1), (32000, 0)):
            h = self.moving()
            with self.subTest(left=left, right=right):
                self.assertEqual(h.drive(left, right).fields[1], p.REASON_RANGE)
                self.assertEqual(h.out, (0, 0))
                self.assertEqual(h.mcu.state, p.STATE_ARMED)

    def test_bad_args_are_nakd_and_stop_a_moving_robot(self):
        bad_sets = [("1.5", "0", "100"), ("abc", "0", "100"), ("+5", "0", "100"),
                    ("1", "2"), ("1", "2", "3", "4"), ("123456", "0", "100"), ("--1", "0", "100")]
        for args in bad_sets:
            h = self.moving()
            with self.subTest(args=args):
                reply = h.one("DRV", *args)
                self.assertEqual(reply.fields[1], p.REASON_BAD_ARGS)
                self.assertEqual(h.out, (0, 0))

    def test_drv_with_unparseable_sequence_also_fails_safe(self):
        h = self.moving()
        replies = h.raw(p.encode("DRV", "abc", 0, 0, 100))
        self.assertEqual(replies, [p.Frame("NAK", ("0", p.REASON_BAD_ARGS))])
        self.assertEqual(h.out, (0, 0))

    def test_bad_sequence_numbers(self):
        h = Harness()
        for seq in ("abc", "-1", "65536", "123456", "1.0"):
            with self.subTest(seq=seq):
                self.assertEqual(h.raw(p.encode("PING", seq)), [p.Frame("NAK", ("0", p.REASON_BAD_ARGS))])
        self.assertEqual(h.raw(p.encode("PING", 65535)), [p.Frame("ACK", ("65535", "DISARMED"))])
        self.assertEqual(h.raw(p.encode("PING", 0)), [p.Frame("ACK", ("0", "DISARMED"))])
        self.assertEqual(h.raw(p.encode("PING")), [p.Frame("NAK", ("0", p.REASON_BAD_ARGS))])

    def test_commands_with_unexpected_arguments_are_rejected(self):
        h = Harness()
        for cmd in ("HELLO", "ARM", "DISARM", "STOP", "PING", "STAT"):
            with self.subTest(cmd=cmd):
                self.assertEqual(h.raw(p.encode(cmd, 1, "extra")), [p.Frame("NAK", ("1", p.REASON_BAD_ARGS))])
        self.assertEqual(h.mcu.state, p.STATE_DISARMED)  # a malformed ARM must not arm

    def test_unknown_command_is_nakd_and_does_not_disturb_motion(self):
        h = self.moving()
        self.assertEqual(h.raw(p.encode("FOO", 5)), [p.Frame("NAK", ("5", p.REASON_UNKNOWN_CMD))])
        self.assertEqual(h.raw(p.encode("FOO")), [p.Frame("NAK", ("0", p.REASON_UNKNOWN_CMD))])
        self.assertEqual(h.out, (300, 300))

    def test_bad_frame_is_nakd_without_state_change(self):
        h = self.moving()
        good = p.encode("PING", 4)
        bad = bytearray(good)
        bad[2] ^= 0x01
        self.assertEqual(h.raw(bytes(bad)), [p.Frame("NAK", ("0", p.REASON_BAD_FRAME))])
        self.assertEqual((h.mcu.state, h.out), (p.STATE_ARMED, (300, 300)))

    def test_oversized_frame_is_nakd(self):
        h = Harness()
        replies = h.raw(b"$" + b"A" * 100 + b"\n" + p.encode("PING", 2))
        self.assertEqual(replies[0], p.Frame("NAK", ("0", p.REASON_BAD_FRAME)))
        self.assertEqual(replies[-1], p.Frame("ACK", ("2", "DISARMED")))


class StreamHandlingTests(unittest.TestCase):
    def test_byte_by_byte_delivery(self):
        h = Harness()
        wire = p.encode("ARM", 1) + p.encode("DRV", 2, 100, 100, 250)
        out = b""
        for i in range(len(wire)):
            out += h.mcu.feed(wire[i:i + 1])
        self.assertEqual(p.FrameAssembler().feed(out),
                         [p.Frame("ACK", ("1", "ARMED")), p.Frame("ACK", ("2", "ARMED"))])
        self.assertEqual(h.out, (100, 100))

    def test_multiple_frames_in_one_chunk_answered_in_order(self):
        h = Harness()
        replies = h.raw(p.encode("PING", 1) + p.encode("ARM", 2) + p.encode("PING", 3))
        self.assertEqual([r.fields for r in replies],
                         [("1", "DISARMED"), ("2", "ARMED"), ("3", "ARMED")])

    def test_replies_are_valid_frames_within_the_size_limit(self):
        h = Harness()
        h.arm()
        h.drive(-CAP, -CAP, p.MAX_TTL_MS)
        for cmd, fields in (("STAT", ()), ("HELLO", ()), ("PING", ()), ("DRV", (1, 1, 1))):
            wire = h.mcu.feed(p.encode(cmd, 65535, *fields))
            self.assertLessEqual(len(wire), p.MAX_FRAME_BYTES)
            self.assertIsInstance(p.decode(wire), p.Frame)

    def test_worst_case_stat_frame_fits(self):
        h = Harness(link_timeout_ms=1_000_000_000)
        h.arm()
        h.advance(2**32 - 1 - 10)
        h.drive(-CAP, -CAP, p.MAX_TTL_MS)
        h.advance(10)
        wire = h.mcu.feed(p.encode("STAT", 65535))
        self.assertLessEqual(len(wire), p.MAX_FRAME_BYTES)
        self.assertEqual(p.decode(wire).fields[-1], str(2**32 - 1))

    def test_uptime_wraps_like_a_32_bit_counter(self):
        h = Harness(link_timeout_ms=1_000_000_000_000)
        h.advance(2**32 + 5)
        self.assertEqual(h.one("STAT").fields[-1], "5")

    def test_deterministic_replay(self):
        def run():
            h = Harness()
            wire = b""
            for step in (p.encode("ARM", 1), p.encode("DRV", 2, 10, -10, 100), p.encode("STAT", 3)):
                wire += h.mcu.feed(step)
                h.advance(60)
            return wire, h.mcu.history
        self.assertEqual(run(), run())

    def test_history_records_only_changes(self):
        h = Harness()
        h.arm()
        h.drive(100, 100)
        h.drive(100, 100)   # identical: no new entry
        h.one("STOP")
        h.one("STOP")
        self.assertEqual([(l, r) for _, l, r in h.mcu.history], [(0, 0), (100, 100), (0, 0)])


class InvariantFuzzTests(unittest.TestCase):
    """Random traffic and random time gaps; safety invariants must hold after every step."""

    def test_safety_invariants_under_random_traffic(self):
        rng = random.Random(20261015)
        for trial in range(60):
            h = Harness()
            for step in range(150):
                op = rng.random()
                if op < 0.12:
                    h.send("ARM")
                elif op < 0.20:
                    h.send("DISARM")
                elif op < 0.28:
                    h.send("STOP")
                elif op < 0.60:
                    h.send("DRV", rng.randint(-700, 700), rng.randint(-700, 700), rng.randint(-5, 1200))
                elif op < 0.70:
                    h.send("PING")
                elif op < 0.76:
                    h.raw(bytes(rng.randrange(256) for _ in range(rng.randint(0, 40))))  # line noise
                elif op < 0.80:
                    wire = bytearray(p.encode("DRV", 1, 100, 100, 250))
                    wire[rng.randrange(1, len(wire) - 1)] ^= 0x04
                    h.raw(bytes(wire))
                h.advance(rng.choice([0, 1, 10, 50, 120, 249, 250, 251, 499, 500, 501, 900]))
                h.mcu.tick()

                left, right = h.out
                where = f"trial={trial} step={step}"
                self.assertLessEqual(max(abs(left), abs(right)), CAP, where)
                if h.mcu.state == p.STATE_DISARMED:
                    self.assertEqual((left, right), (0, 0), where)
                if (left, right) != (0, 0):
                    self.assertEqual(h.mcu.state, p.STATE_ARMED, where)
                    self.assertGreater(h.mcu.ttl_remaining_ms, 0, where)
                if h.mcu.state == p.STATE_ARMED:
                    self.assertLess(h.clock.now_ms() - h.last_valid_ms, LINK_TIMEOUT, where)
                if h.mcu.state == p.STATE_ARMED:
                    self.assertEqual(h.mcu.fault, p.FAULT_NONE, where)


if __name__ == "__main__":
    unittest.main()

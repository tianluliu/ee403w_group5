"""Host-side bridge tests against the mock MCU through the fault-injecting loopback.

No serial port, no hardware, no sleeping (FakeClock). Hand-checked drive value: with the mock
drivetrain, a 0.1 m/s request becomes 426 permille on both wheels (see test_kinematics).
"""

import dataclasses
import math
import random
import unittest

import _bootstrap
from lidar_robot import protocol as p
from lidar_robot.bridge import (
    BridgeError, ConfigSafetyError, LinkError, McuStatus, MotorBridge, MotorRejected,
    MotorsDisabledError, NotArmedError, ProtocolMismatch, TransportDown,
)
from lidar_robot.clock import FakeClock
from lidar_robot.config import IncompleteHardwareConfig, Safety, load_config
from lidar_robot.kinematics import Twist
from lidar_robot.mock_mcu import MockMotorController
from lidar_robot.transport import LoopbackTransport, Transport

MOCK = load_config(_bootstrap.CONFIG_DIR / "robot.mock.toml")
REAL = load_config(_bootstrap.CONFIG_DIR / "robot.toml")
ENABLED = dataclasses.replace(MOCK, safety=Safety(motors_enabled=True))
CAP = MOCK.limits.max_duty_permille


class ScriptedMcu:
    """Stand-in MCU whose replies are produced by a function (for malformed-reply tests)."""

    def __init__(self, responder):
        self._responder = responder
        self._asm = p.FrameAssembler()
        self.seen = []

    def feed(self, data):
        out = b""
        for item in self._asm.feed(data):
            if isinstance(item, p.Frame):
                self.seen.append(item)
                out += self._responder(item) or b""
        return out

    def tick(self):
        pass


def ack(frame, state="DISARMED"):
    return p.encode("ACK", frame.fields[0], state)


class Rig:
    def __init__(self, cfg=ENABLED, mcu=None, mcu_kwargs=None, chunk=None, **bridge_kw):
        self.cfg = cfg
        self.clock = FakeClock()
        if mcu is None:
            kw = dict(link_timeout_ms=cfg.limits.link_timeout_ms,
                      max_duty_permille=cfg.limits.max_duty_permille)
            kw.update(mcu_kwargs or {})
            mcu = MockMotorController(self.clock, **kw)
        self.mcu = mcu
        self.transport = LoopbackTransport(mcu, read_chunk=chunk)
        self.bridge = MotorBridge(self.transport, cfg, self.clock, **bridge_kw)

    def armed(self):
        self.bridge.connect()
        self.bridge.arm()
        return self

    def sent(self):
        return [p.decode(w) for w in self.transport.write_log]

    def sent_cmds(self):
        return [f.cmd for f in self.sent()]


class MotorsDisabledByDefaultTests(unittest.TestCase):
    def test_arm_and_drive_are_refused_before_any_byte_is_written(self):
        rig = Rig(cfg=MOCK)
        rig.bridge.connect()                       # handshake is harmless and allowed
        before = len(rig.transport.write_log)
        with self.assertRaises(MotorsDisabledError):
            rig.bridge.arm()
        with self.assertRaises(MotorsDisabledError):
            rig.bridge.send_twist(Twist(0.1, 0.0))
        with self.assertRaises(MotorsDisabledError):
            rig.bridge.send_twist(Twist(0.0, 0.0))  # even a zero command
        self.assertEqual(len(rig.transport.write_log), before)
        self.assertEqual((rig.mcu.state, rig.mcu.outputs), (p.STATE_DISARMED, (0, 0)))

    def test_repository_configs_are_disabled(self):
        for cfg in (MOCK, REAL):
            rig = Rig(cfg=cfg)
            with self.assertRaises(MotorsDisabledError):
                rig.bridge.arm()
            self.assertEqual(rig.transport.write_log, [])

    def test_safe_commands_remain_available_while_disabled(self):
        rig = Rig(cfg=MOCK)
        self.assertTrue(rig.bridge.stop())
        self.assertTrue(rig.bridge.disarm())
        self.assertEqual(rig.sent_cmds(), ["STOP", "DISARM"])


class ConnectTests(unittest.TestCase):
    def test_connect_reports_mcu_info_and_never_arms(self):
        rig = Rig()
        info = rig.bridge.connect()
        self.assertEqual((info.proto, info.fw_id, info.caps, info.state), (1, "mock-0.1", 0, "DISARMED"))
        self.assertFalse(info.has_encoders)
        self.assertEqual(rig.bridge.mcu_state, p.STATE_DISARMED)
        self.assertEqual(rig.mcu.state, p.STATE_DISARMED)
        self.assertEqual(rig.sent()[0], p.Frame("HELLO", ("1",)))

    def test_stale_bytes_from_before_connect_are_discarded(self):
        rig = Rig()
        rig.transport.inject_noise(p.encode("ACK", 1, "ARMED"))   # same seq as the HELLO
        self.assertEqual(rig.bridge.connect().state, "DISARMED")
        self.assertEqual(rig.bridge.stats.stale_replies, 0)

    def test_arm_requires_connect_first(self):
        rig = Rig()
        with self.assertRaises(BridgeError):
            rig.bridge.arm()
        self.assertEqual(rig.transport.write_log, [])

    def test_protocol_version_mismatch(self):
        mcu = ScriptedMcu(lambda f: p.encode("HELLO", f.fields[0], 2, "x", "00", "DISARMED"))
        with self.assertRaises(ProtocolMismatch):
            Rig(mcu=mcu).bridge.connect()

    def test_malformed_hello_replies(self):
        bad = [
            lambda f: p.encode("HELLO", f.fields[0], 1, "x", "00"),                 # too few fields
            lambda f: p.encode("HELLO", f.fields[0], 1, "x", "ZZ", "DISARMED"),     # caps not hex
            lambda f: p.encode("HELLO", f.fields[0], 1, "x", "0", "DISARMED"),      # caps too short
            lambda f: p.encode("HELLO", f.fields[0], 1, "x", "00", "EXPLODING"),    # unknown state
            lambda f: p.encode("HELLO", f.fields[0], "one", "x", "00", "DISARMED"), # proto not numeric
            lambda f: p.encode("NAK", f.fields[0], "UNKNOWN_CMD"),                  # rejected
            lambda f: ack(f),                                                       # wrong reply type
        ]
        for responder in bad:
            with self.subTest():
                with self.assertRaises(ProtocolMismatch):
                    Rig(mcu=ScriptedMcu(responder)).bridge.connect()

    def test_encoder_capability_consistency(self):
        yes = dataclasses.replace(ENABLED, encoders=dataclasses.replace(ENABLED.encoders, present="yes"))
        with self.assertRaises(ProtocolMismatch):                 # config says yes, MCU says no
            Rig(cfg=yes).bridge.connect()
        info = Rig(cfg=yes, mcu_kwargs={"caps": p.CAP_ENCODERS}).bridge.connect()
        self.assertTrue(info.has_encoders)
        info = Rig(cfg=ENABLED, mcu_kwargs={"caps": p.CAP_ENCODERS}).bridge.connect()  # config "no"
        self.assertTrue(info.has_encoders)                        # reported, not assumed

    def test_handshake_retries_but_gives_up(self):
        rig = Rig()
        rig.transport.drop_writes = 2
        self.assertEqual(rig.bridge.connect().proto, 1)
        rig = Rig()
        rig.transport.drop_writes = 3
        with self.assertRaises(LinkError):
            rig.bridge.connect()
        self.assertEqual(rig.bridge.stats.timeouts, 3)


class ArmAndDriveTests(unittest.TestCase):
    def test_hand_checked_drive_through_the_whole_stack(self):
        rig = Rig().armed()
        cmd = rig.bridge.send_twist(Twist(0.1, 0.0))
        self.assertEqual((cmd.left_permille, cmd.right_permille), (426, 426))
        self.assertEqual(rig.mcu.outputs, (426, 426))
        drv = rig.sent()[-1]
        self.assertEqual((drv.cmd, drv.fields[1:]), ("DRV", ("426", "426", str(MOCK.limits.cmd_ttl_ms))))
        self.assertEqual(rig.bridge.mcu_state, p.STATE_ARMED)

    def test_motion_stops_by_itself_when_the_host_goes_silent(self):
        rig = Rig().armed()
        rig.bridge.send_twist(Twist(0.1, 0.0))
        rig.clock.advance(MOCK.limits.cmd_ttl_ms - 1)
        rig.mcu.tick()
        self.assertEqual(rig.mcu.outputs, (426, 426))
        rig.clock.advance(1)
        rig.mcu.tick()
        self.assertEqual(rig.mcu.outputs, (0, 0))
        rig.clock.advance(MOCK.limits.link_timeout_ms)
        rig.mcu.tick()
        self.assertEqual((rig.mcu.state, rig.mcu.fault), (p.STATE_DISARMED, p.FAULT_LINK_TIMEOUT))

    def test_regular_commands_keep_the_robot_moving(self):
        rig = Rig().armed()
        for _ in range(40):                                   # 4 s at 10 Hz
            rig.bridge.send_twist(Twist(0.1, 0.0))
            rig.clock.advance(MOCK.limits.cmd_period_ms)
            rig.mcu.tick()
            self.assertEqual(rig.mcu.outputs, (426, 426))
        self.assertEqual(rig.mcu.state, p.STATE_ARMED)

    def test_drive_before_arm_is_refused_without_sending(self):
        rig = Rig()
        rig.bridge.connect()
        n = len(rig.transport.write_log)
        with self.assertRaises(NotArmedError):
            rig.bridge.send_twist(Twist(0.1, 0.0))
        self.assertEqual(len(rig.transport.write_log), n)

    def test_stop_keeps_the_mcu_armed_and_disarm_does_not(self):
        rig = Rig().armed()
        rig.bridge.send_twist(Twist(0.1, 0.0))
        self.assertTrue(rig.bridge.stop())
        self.assertEqual((rig.mcu.outputs, rig.mcu.state), ((0, 0), p.STATE_ARMED))
        rig.bridge.send_twist(Twist(0.1, 0.0))
        self.assertTrue(rig.bridge.disarm())
        self.assertEqual((rig.mcu.outputs, rig.mcu.state), ((0, 0), p.STATE_DISARMED))
        with self.assertRaises(NotArmedError):
            rig.bridge.send_twist(Twist(0.1, 0.0))

    def test_requests_beyond_limits_are_clamped_before_sending(self):
        rig = Rig().armed()
        cmd = rig.bridge.send_twist(Twist(50.0, 90.0))
        self.assertLessEqual(max(abs(cmd.left_permille), abs(cmd.right_permille)), CAP)
        left, right = (int(x) for x in rig.sent()[-1].fields[1:3])
        self.assertLessEqual(max(abs(left), abs(right)), CAP)

    def test_non_finite_twist_raises_and_stops_the_motors(self):
        for bad in (Twist(float("nan"), 0.0), Twist(0.0, float("inf"))):
            rig = Rig().armed()
            rig.bridge.send_twist(Twist(0.1, 0.0))
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    rig.bridge.send_twist(bad)
                self.assertEqual(rig.mcu.outputs, (0, 0))
                self.assertEqual(rig.sent_cmds()[-1], "STOP")

    def test_incomplete_hardware_config_refuses_to_arm_and_sends_nothing_more(self):
        cfg = dataclasses.replace(REAL, safety=Safety(motors_enabled=True))   # real values: unset
        rig = Rig(cfg=cfg)
        rig.bridge.connect()
        with self.assertRaises(IncompleteHardwareConfig) as ctx:
            rig.bridge.arm()
        self.assertIn("drivetrain.wheel_radius_m", ctx.exception.missing)
        self.assertEqual(rig.sent_cmds(), ["HELLO"])
        self.assertEqual(rig.mcu.state, p.STATE_DISARMED)

    def test_incomplete_config_on_the_drive_path_fails_safe(self):
        cfg = dataclasses.replace(REAL, safety=Safety(motors_enabled=True))
        rig = Rig(cfg=cfg)
        rig.mcu.feed(p.encode("ARM", 99))          # something else armed the MCU
        rig.bridge.connect()
        self.assertEqual(rig.bridge.mcu_state, p.STATE_ARMED)
        with self.assertRaises(IncompleteHardwareConfig):
            rig.bridge.send_twist(Twist(0.1, 0.0))
        self.assertEqual(rig.sent_cmds(), ["HELLO", "STOP"])
        self.assertEqual(rig.mcu.outputs, (0, 0))

    def test_mcu_side_range_rejection_surfaces_and_leaves_motors_stopped(self):
        rig = Rig(mcu_kwargs={"max_duty_permille": 100}).armed()   # MCU cap tighter than host cap
        with self.assertRaises(MotorRejected) as ctx:
            rig.bridge.send_twist(Twist(0.1, 0.0))
        self.assertEqual(ctx.exception.reason, p.REASON_RANGE)
        self.assertEqual(rig.mcu.outputs, (0, 0))
        self.assertEqual(rig.bridge.stats.naks, 1)

    def test_mcu_disarmed_by_link_timeout_is_detected_on_next_command(self):
        rig = Rig().armed()
        rig.bridge.send_twist(Twist(0.1, 0.0))
        rig.clock.advance(MOCK.limits.link_timeout_ms)       # host was silent too long
        with self.assertRaises(MotorRejected) as ctx:
            rig.bridge.send_twist(Twist(0.1, 0.0))
        self.assertEqual(ctx.exception.reason, p.REASON_NOT_ARMED)
        self.assertEqual(rig.bridge.mcu_state, p.STATE_DISARMED)
        self.assertEqual(rig.mcu.outputs, (0, 0))
        n = len(rig.transport.write_log)
        with self.assertRaises(NotArmedError):               # and is not retried blindly
            rig.bridge.send_twist(Twist(0.1, 0.0))
        self.assertEqual(len(rig.transport.write_log), n)

    def test_stat_and_ping(self):
        rig = Rig().armed()
        rig.bridge.send_twist(Twist(0.1, 0.0))
        rig.clock.advance(50)
        rig.bridge.ping()
        self.assertEqual(rig.bridge.stat(), McuStatus("ARMED", 426, 426, 200, "NONE", 50))

    def test_malformed_stat_replies(self):
        for responder in (lambda f: ack(f),
                          lambda f: p.encode("STAT", f.fields[0], "ARMED", 1, 2, 3, "NONE"),
                          lambda f: p.encode("STAT", f.fields[0], "ARMED", "x", 2, 3, "NONE", 5)):
            with self.subTest():
                with self.assertRaises(ProtocolMismatch):
                    Rig(mcu=ScriptedMcu(responder)).bridge.stat()


class LinkFailureTests(unittest.TestCase):
    def test_disconnect_is_reported_and_stop_never_raises(self):
        rig = Rig().armed()
        rig.bridge.send_twist(Twist(0.1, 0.0))
        rig.transport.connected = False
        with self.assertRaises(TransportDown):
            rig.bridge.send_twist(Twist(0.1, 0.0))
        self.assertIsInstance(TransportDown("x"), LinkError)
        self.assertFalse(rig.bridge.stop())                  # cannot deliver, but no exception
        self.assertFalse(rig.bridge.disarm())
        self.assertIsNotNone(rig.bridge.last_error)
        # The MCU is the backstop: TTL then link timeout, with no help from the host.
        rig.clock.advance(MOCK.limits.cmd_ttl_ms)
        rig.mcu.tick()
        self.assertEqual(rig.mcu.outputs, (0, 0))
        rig.clock.advance(MOCK.limits.link_timeout_ms)
        rig.mcu.tick()
        self.assertEqual((rig.mcu.state, rig.mcu.fault), (p.STATE_DISARMED, p.FAULT_LINK_TIMEOUT))

    def test_lost_request_times_out_after_exactly_the_reply_timeout(self):
        rig = Rig().armed()
        rig.transport.drop_writes = 1
        t0 = rig.clock.now_ms()
        with self.assertRaises(LinkError):
            rig.bridge.ping()
        self.assertEqual(rig.clock.now_ms() - t0, 50)
        self.assertEqual(rig.bridge.stats.timeouts, 1)

    def test_lost_drive_request_stops_the_robot(self):
        rig = Rig().armed()
        rig.bridge.send_twist(Twist(0.1, 0.0))
        rig.transport.drop_writes = 1
        with self.assertRaises(LinkError):
            rig.bridge.send_twist(Twist(0.1, 0.0))
        self.assertEqual(rig.mcu.outputs, (0, 0))
        self.assertEqual(rig.sent_cmds()[-1], "STOP")

    def test_lost_reply_means_the_mcu_did_act_but_the_host_still_stops_it(self):
        rig = Rig().armed()
        rig.transport.drop_replies = 1
        with self.assertRaises(LinkError):
            rig.bridge.send_twist(Twist(0.1, 0.0))
        self.assertIn((426, 426), [(l, r) for _, l, r in rig.mcu.history[1:]])   # it did move
        self.assertEqual(rig.mcu.outputs, (0, 0))                                # and was stopped

    def test_corrupted_reply_is_ignored_and_counted(self):
        rig = Rig().armed()
        rig.transport.corrupt_replies = 1
        with self.assertRaises(LinkError):
            rig.bridge.ping()
        self.assertGreaterEqual(rig.bridge.stats.bad_frames, 1)
        rig.bridge.ping()                                    # link recovers

    def test_mcu_reporting_a_corrupt_request_fails_fast_and_stops(self):
        def responder(f):
            if f.cmd == "DRV":
                return p.encode("NAK", 0, p.REASON_BAD_FRAME)
            return ack(f, "ARMED") if f.cmd in ("STOP", "ARM") else p.encode("HELLO", f.fields[0], 1, "x", "00", "ARMED")
        rig = Rig(mcu=ScriptedMcu(responder))
        rig.bridge.connect()
        rig.bridge.arm()
        t0 = rig.clock.now_ms()
        with self.assertRaises(LinkError):
            rig.bridge.send_twist(Twist(0.1, 0.0))
        self.assertLess(rig.clock.now_ms() - t0, 50)         # did not wait for the full timeout
        self.assertEqual(rig.bridge.stats.bad_frame_naks, 1)
        self.assertEqual(rig.sent_cmds()[-1], "STOP")

    def test_line_noise_and_fragmented_reads_are_tolerated(self):
        rig = Rig(chunk=1)
        rig.transport.inject_noise(b"\x00\xffjunk$garbage\n")
        rig.bridge.connect()
        rig.bridge.arm()
        rig.bridge.send_twist(Twist(0.1, 0.0))
        self.assertEqual(rig.mcu.outputs, (426, 426))

    def test_stale_reply_is_skipped_not_mistaken_for_the_answer(self):
        def responder(f):
            return ack(f) if f.cmd != "PING" else (
                p.encode("ACK", int(f.fields[0]) - 1, "ARMED") + p.encode("ACK", f.fields[0], "DISARMED"))
        rig = Rig(mcu=ScriptedMcu(responder))
        rig.bridge.ping()
        self.assertEqual(rig.bridge.stats.stale_replies, 1)
        self.assertEqual(rig.bridge.mcu_state, p.STATE_DISARMED)   # taken from the matching reply

    def test_unknown_state_in_ack_is_a_mismatch(self):
        rig = Rig(mcu=ScriptedMcu(lambda f: p.encode("ACK", f.fields[0], "BANANAS")))
        with self.assertRaises(ProtocolMismatch):
            rig.bridge.ping()

    def test_stop_is_retried_and_reports_failure_honestly(self):
        rig = Rig()
        rig.transport.drop_writes = 2
        self.assertTrue(rig.bridge.stop())
        self.assertEqual(rig.sent_cmds(), ["STOP"] * 3)
        rig = Rig()
        rig.transport.drop_writes = 3
        self.assertFalse(rig.bridge.stop())
        self.assertIn("no valid reply", rig.bridge.last_error)

    def test_stop_returns_false_when_mcu_rejects_it(self):
        rig = Rig(mcu=ScriptedMcu(lambda f: p.encode("NAK", f.fields[0], "RANGE")))
        self.assertFalse(rig.bridge.stop())

    def test_drive_commands_are_not_retried(self):
        rig = Rig().armed()
        rig.transport.drop_writes = 1
        with self.assertRaises(LinkError):
            rig.bridge.send_twist(Twist(0.1, 0.0))
        self.assertEqual(rig.sent_cmds().count("DRV"), 1)

    def test_keepalive_pings_hold_the_link_and_silence_drops_it(self):
        rig = Rig().armed()
        for _ in range(25):                                  # 10 s of pings every 400 ms
            rig.clock.advance(400)
            rig.bridge.ping()
        self.assertEqual(rig.mcu.state, p.STATE_ARMED)
        rig.clock.advance(MOCK.limits.link_timeout_ms)
        rig.mcu.tick()
        self.assertEqual(rig.mcu.state, p.STATE_DISARMED)

    def test_close_disarms_first_and_is_idempotent(self):
        rig = Rig().armed()
        rig.bridge.send_twist(Twist(0.1, 0.0))
        rig.bridge.close()
        self.assertEqual(rig.sent_cmds()[-1], "DISARM")
        self.assertEqual((rig.mcu.state, rig.mcu.outputs), (p.STATE_DISARMED, (0, 0)))
        self.assertFalse(rig.transport.is_open)
        rig.bridge.close()                                   # second close must not raise
        with self.assertRaises(TransportDown):
            rig.bridge.ping()


class SequenceNumberTests(unittest.TestCase):
    def test_sequence_increments_from_one(self):
        rig = Rig()
        for _ in range(3):
            rig.bridge.ping()
        self.assertEqual([f.fields[0] for f in rig.sent()], ["1", "2", "3"])

    def test_sequence_wraps_to_one_and_never_uses_zero(self):
        rig = Rig(first_seq=65534)
        for _ in range(4):
            rig.bridge.ping()
        self.assertEqual([f.fields[0] for f in rig.sent()], ["65534", "65535", "1", "2"])

    def test_first_seq_validation(self):
        for bad in (0, -1, 65536):
            with self.assertRaises(ValueError):
                Rig(first_seq=bad)


class ConstructionSafetyTests(unittest.TestCase):
    class RealishTransport(Transport):
        """Claims to reach hardware (is_simulated False); never touches any."""
        is_simulated = property(lambda self: False)
        def write(self, data): raise AssertionError("must not be reached")
        def read(self): return b""
        def close(self): pass

    def test_mock_config_is_refused_on_a_non_simulated_transport(self):
        with self.assertRaises(ConfigSafetyError):
            MotorBridge(self.RealishTransport(), ENABLED, FakeClock())

    def test_real_config_is_accepted_on_a_non_simulated_transport(self):
        MotorBridge(self.RealishTransport(), REAL, FakeClock())     # construction only; no I/O

    def test_argument_validation(self):
        t = LoopbackTransport(MockMotorController(FakeClock()))
        for kw in ({"reply_timeout_ms": 0}, {"poll_ms": 0}, {"reply_timeout_ms": 10, "poll_ms": 11},
                   {"stop_attempts": 0}):
            with self.subTest(kw=kw):
                with self.assertRaises(ValueError):
                    MotorBridge(t, ENABLED, FakeClock(), **kw)
        with self.assertRaises(ValueError):
            LoopbackTransport(MockMotorController(FakeClock()), read_chunk=0)


class FaultInjectionFuzzTests(unittest.TestCase):
    """Random operations and faults. Only BridgeError/ValueError may escape; MCU and wire
    invariants must hold at every step."""

    def run_fuzz(self, cfg, seed, steps=200):
        rng = random.Random(seed)
        rig = Rig(cfg=cfg)
        twists = [Twist(0.1, 0.0), Twist(0.0, 0.5), Twist(9.0, 9.0), Twist(-0.1, 0.0),
                  Twist(float("nan"), 0.0), Twist(0.0, float("inf")), Twist(0.0, 0.0)]
        for step in range(steps):
            op = rng.randrange(9)
            try:
                if op == 0: rig.bridge.connect()
                elif op == 1: rig.bridge.arm()
                elif op in (2, 3, 4): rig.bridge.send_twist(rng.choice(twists))
                elif op == 5: rig.bridge.stop()
                elif op == 6: rig.bridge.ping()
                elif op == 7: rig.bridge.stat()
                else: rig.bridge.disarm()
            except (BridgeError, ValueError):
                pass
            rig.transport.connected = rng.random() > 0.05
            rig.transport.drop_writes = rng.choice([0, 0, 0, 1])
            rig.transport.drop_replies = rng.choice([0, 0, 0, 1])
            rig.transport.corrupt_replies = rng.choice([0, 0, 0, 1])
            rig.clock.advance(rng.choice([0, 10, 100, 249, 250, 500, 600]))
            rig.mcu.tick()
            left, right = rig.mcu.outputs
            where = f"seed={seed} step={step}"
            self.assertLessEqual(max(abs(left), abs(right)), CAP, where)
            if rig.mcu.state == p.STATE_DISARMED:
                self.assertEqual((left, right), (0, 0), where)
            if (left, right) != (0, 0):
                self.assertGreater(rig.mcu.ttl_remaining_ms, 0, where)
        return rig

    def test_motors_enabled_invariants(self):
        for seed in range(25):
            rig = self.run_fuzz(ENABLED, seed)
            for frame in rig.sent():
                if frame.cmd == "DRV":
                    left, right, ttl = (int(x) for x in frame.fields[1:4])
                    self.assertLessEqual(max(abs(left), abs(right)), CAP)
                    self.assertEqual(ttl, MOCK.limits.cmd_ttl_ms)
            for raw in rig.transport.write_log:
                self.assertLessEqual(len(raw), p.MAX_FRAME_BYTES)

    def test_motors_disabled_means_no_arm_or_drive_frame_is_ever_written(self):
        for seed in range(25):
            rig = self.run_fuzz(MOCK, seed)
            self.assertFalse({"ARM", "DRV"} & set(rig.sent_cmds()), f"seed={seed}")
            self.assertEqual(rig.mcu.history[1:], [(t, 0, 0) for (t, _, _) in rig.mcu.history[1:]])


if __name__ == "__main__":
    unittest.main()

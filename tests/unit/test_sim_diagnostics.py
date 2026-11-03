"""Instrumentation must observe deadlines without refreshing or evaluating them."""
import ast
from dataclasses import replace
from types import SimpleNamespace, MethodType
from unittest import TestCase
from unittest.mock import Mock

from _bootstrap import REPO_ROOT, CONFIG_DIR
from lidar_robot import protocol as p
from lidar_robot.clock import FakeClock
from lidar_robot.mock_mcu import MockMotorController
from lidar_robot.sim_diagnostics import SimulatorDiagnostics
from lidar_robot.config import load_config
from lidar_robot.bridge import MotorBridge
from lidar_robot.transport import LoopbackTransport
from lidar_robot.kinematics import Twist


class WindowTests(TestCase):
    def test_interval_and_duration_extrema_survive_delayed_publication(self):
        d = SimulatorDiagnostics()
        d.start('control', 100)
        d.finish('control', 100, 110)
        d.start('control', 700)
        d.finish('control', 700, 730)
        row = d.drain()['timers']['control']
        self.assertEqual(row['max_interval_ns'], 600)
        self.assertEqual(row['max_interval_start_ns'], 100)
        self.assertEqual(row['max_interval_end_ns'], 700)
        self.assertEqual(row['max_duration_ns'], 30)
        d.start('control', 800)
        self.assertEqual(d.drain()['timers']['control']['last_interval_ns'], 100)

    def test_bounded_events_counts_and_rejection_coalescing(self):
        d = SimulatorDiagnostics(max_events=2)
        for t in (1, 2):
            d.event(dict(event='drive_rejected', reason='NOT_ARMED', monotonic_ms=t))
        d.event(dict(event='arm', monotonic_ms=3))
        row = d.drain()
        self.assertEqual(row['events'][0]['repeats'], 2)
        self.assertEqual(row['events'][0]['last_monotonic_ms'], 2)
        self.assertEqual(row['event_counts']['drive_rejected'], 2)
        for t in range(3):
            d.event(dict(event='stop', monotonic_ms=t))
        row = d.drain()
        self.assertEqual(len(row['events']), 2)
        self.assertEqual(row['dropped_events'], 1)


class McuObservationTests(TestCase):
    def test_snapshot_does_not_tick_or_refresh_watchdog(self):
        c = FakeClock()
        m = MockMotorController(c)
        m.feed(p.encode('ARM', 1))
        c.advance(501)
        state = m.diagnostic_state()
        self.assertEqual(state['state'], p.STATE_ARMED)
        self.assertEqual(state['link_age_ms'], 501)
        self.assertEqual(state['link_remaining_ms'], 0)
        m.tick()
        self.assertEqual(m.state, p.STATE_DISARMED)
        self.assertEqual(m.fault, p.FAULT_LINK_TIMEOUT)

    def test_expiry_observed_before_late_frame_refresh_and_rejection(self):
        c = FakeClock()
        events = []
        m = MockMotorController(c, observer=events.append)
        m.feed(p.encode('ARM', 1))
        m.feed(p.encode('DRV', 2, 100, 100, 250))
        c.advance(501)
        reply = m.feed(p.encode('DRV', 3, 100, 100, 250))
        self.assertIn(b'NOT_ARMED', reply)
        self.assertEqual([e['event'] for e in events], ['arm', 'watchdog_disarm', 'drive_rejected'])
        expired = events[1]
        self.assertEqual(expired['monotonic_ms'], 501)
        self.assertEqual(expired['mcu']['link_age_ms'], 501)
        self.assertEqual(expired['mcu']['outputs'], [0, 0])
        self.assertEqual(events[2]['mcu']['link_age_ms'], 0)

    def test_stop_edges_ttl_and_wire_replies_unchanged(self):
        def run(observer):
            c = FakeClock()
            m = MockMotorController(c, observer=observer)
            replies = [m.feed(p.encode('ARM', 1)), m.feed(p.encode('DRV', 2, 100, 100, 250))]
            c.advance(250)
            m.tick()
            replies += [m.feed(p.encode('STOP', 3)), m.feed(p.encode('STOP', 4))]
            return replies, m.history, m.state, m.fault
        events = []
        self.assertEqual(run(None), run(events.append))
        self.assertEqual([e['event'] for e in events], ['arm', 'command_ttl_expired', 'stop'])
        self.assertEqual(events[-1]['mcu']['command_counts']['STOP'], 1)

    def test_observer_exception_cannot_change_safety_or_protocol(self):
        def broken(event):
            raise RuntimeError('diagnostic sink failed')
        c = FakeClock()
        m = MockMotorController(c, observer=broken)
        self.assertIn(b'ACK', m.feed(p.encode('ARM', 1)))
        c.advance(500)
        m.tick()
        self.assertEqual(m.state, p.STATE_DISARMED)
        self.assertEqual(m.outputs, (0, 0))
        self.assertEqual(m.diagnostic_state()['observer_errors'], 2)

    def test_invalid_stop_arguments_are_not_reported_as_accepted_stop(self):
        events = []
        m = MockMotorController(FakeClock(), observer=events.append)
        self.assertIn(b'NAK', m.feed(p.encode('STOP', 1, 'extra')))
        self.assertEqual(events, [])


class ControlPathTests(TestCase):
    def test_actual_ros_control_method_never_rearms_latched_session(self):
        # Run the actual control callback with real in-memory bridge/MCU objects;
        # no ROS needed, and diagnostics must not change its safety decisions.
        path = REPO_ROOT / 'ros2_ws/src/robot_mock/robot_mock/mock_robot_node.py'
        cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_control_tick')
        namespace = {}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
        cfg = load_config(CONFIG_DIR / 'robot.mock.toml')
        cfg = replace(cfg, safety=replace(cfg.safety, motors_enabled=True))
        for enabled in (False, True):
            with self.subTest(diagnostics=enabled):
                clock = FakeClock()
                events = []
                mcu = MockMotorController.from_config(cfg, clock, observer=events.append if enabled else None)
                bridge = MotorBridge(LoopbackTransport(mcu), cfg, clock)
                bridge.connect()
                node = SimpleNamespace(_mono_clock=clock, _cfg=cfg, _mcu=mcu, _bridge=bridge,
                    _latest_cmd=Twist(.05, 0), _latest_cmd_ms=0, _motion_session_active=False,
                    _diagnostics=object() if enabled else None, _diagnostic_decision=None,
                    _record_diagnostic=Mock(), get_logger=lambda: Mock())
                tick = MethodType(namespace['_control_tick'], node)
                tick()
                clock.advance(501)
                mcu.tick()
                node._latest_cmd_ms = 501
                tick()
                tick()
                self.assertEqual(mcu.state, p.STATE_DISARMED)
                self.assertTrue(node._motion_session_active)
                if enabled:
                    self.assertEqual(sum(e['event'] == 'arm' for e in events), 1)
                node._latest_cmd = Twist(0, 0)
                tick()
                node._latest_cmd = Twist(.05, 0)
                tick()
                self.assertEqual(mcu.state, p.STATE_ARMED)

    def test_failed_diagnostic_snapshot_cannot_prevent_fail_safe_stop(self):
        path = REPO_ROOT / 'ros2_ws/src/robot_mock/robot_mock/mock_robot_node.py'
        cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
                   and n.name in ('_control_tick', '_record_diagnostic')]
        namespace = {}
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), namespace)
        cfg = load_config(CONFIG_DIR / 'robot.mock.toml')
        bridge = Mock()
        bridge.send_twist.side_effect = RuntimeError('host not armed')
        mcu = Mock()
        mcu.diagnostic_state.side_effect = RuntimeError('broken observer')
        node = SimpleNamespace(_mono_clock=FakeClock(), _cfg=cfg, _mcu=mcu, _bridge=bridge,
            _latest_cmd=Twist(.05, 0), _latest_cmd_ms=0, _motion_session_active=True,
            _diagnostics=SimulatorDiagnostics(), _diagnostic_decision='drive',
            _diagnostic_record_errors=0, _diagnostic_context=lambda: {},
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)),
            get_logger=lambda: Mock())
        node._record_diagnostic = MethodType(namespace['_record_diagnostic'], node)
        MethodType(namespace['_control_tick'], node)()
        bridge.stop.assert_called_once()
        self.assertEqual(node._diagnostic_record_errors, 1)
        bridge.arm.assert_not_called()

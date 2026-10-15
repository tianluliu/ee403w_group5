"""Platform-independent tests for FakeClock / Deadline timing semantics."""

import unittest

import _bootstrap  # noqa: F401
from lidar_robot.clock import Deadline, FakeClock, SystemClock


class FakeClockTests(unittest.TestCase):
    def test_starts_where_told_and_never_moves_by_itself(self):
        self.assertEqual(FakeClock().now_ms(), 0)
        clock = FakeClock(1234)
        self.assertEqual(clock.now_ms(), 1234)
        self.assertEqual(clock.now_ms(), 1234)

    def test_advance_and_sleep(self):
        clock = FakeClock()
        clock.advance(10)
        clock.sleep_ms(5)
        clock.advance(0)
        self.assertEqual(clock.now_ms(), 15)

    def test_cannot_go_backwards(self):
        clock = FakeClock(100)
        with self.assertRaises(ValueError):
            clock.advance(-1)
        with self.assertRaises(ValueError):
            clock.sleep_ms(-1)
        self.assertEqual(clock.now_ms(), 100)

    def test_deterministic_and_independent_instances(self):
        a, b = FakeClock(), FakeClock()
        for step in (3, 0, 250, 1):
            a.advance(step)
            b.advance(step)
        self.assertEqual(a.now_ms(), b.now_ms())
        a.advance(1)
        self.assertNotEqual(a.now_ms(), b.now_ms())


class DeadlineTests(unittest.TestCase):
    def test_boundary_is_inclusive(self):
        clock = FakeClock()
        d = Deadline(clock, 250)
        clock.advance(249)
        self.assertFalse(d.expired())
        self.assertEqual(d.remaining_ms(), 1)
        clock.advance(1)
        self.assertTrue(d.expired())
        self.assertEqual(d.remaining_ms(), 0)

    def test_remaining_never_negative(self):
        clock = FakeClock()
        d = Deadline(clock, 10)
        clock.advance(1000)
        self.assertEqual(d.remaining_ms(), 0)
        self.assertEqual(d.elapsed_ms(), 1000)

    def test_zero_timeout_is_immediately_expired(self):
        self.assertTrue(Deadline(FakeClock(), 0).expired())

    def test_negative_timeout_rejected(self):
        with self.assertRaises(ValueError):
            Deadline(FakeClock(), -1)

    def test_reset_restarts_the_window(self):
        clock = FakeClock()
        d = Deadline(clock, 100)
        clock.advance(99)
        d.reset()
        clock.advance(99)
        self.assertFalse(d.expired())
        clock.advance(1)
        self.assertTrue(d.expired())

    def test_starts_from_current_time_not_zero(self):
        clock = FakeClock(10_000)
        d = Deadline(clock, 50)
        self.assertFalse(d.expired())
        clock.advance(50)
        self.assertTrue(d.expired())

    def test_sleep_on_fake_clock_expires_deadline_without_real_waiting(self):
        clock = FakeClock()
        d = Deadline(clock, 500)
        clock.sleep_ms(500)
        self.assertTrue(d.expired())


class SystemClockTests(unittest.TestCase):
    def test_monotonic_and_sleep(self):
        clock = SystemClock()
        t0 = clock.now_ms()
        clock.sleep_ms(20)
        t1 = clock.now_ms()
        self.assertGreaterEqual(t1 - t0, 19)  # 1 ms tolerance for integer truncation
        self.assertGreaterEqual(clock.now_ms(), t1)


if __name__ == "__main__":
    unittest.main()

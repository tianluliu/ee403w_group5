"""Bounded, ROS-independent simulator observation windows; never controls motion."""
from collections import Counter, deque


class SimulatorDiagnostics:
    def __init__(self, max_events=64):
        self._events = deque(maxlen=max_events)
        self._last_start = {}
        self._timing = {}
        self.counts = Counter()
        self.dropped_events = 0
        self._sequence = 0

    def start(self, name, now_ns):
        previous = self._last_start.get(name)
        self._last_start[name] = now_ns
        row = self._timing.setdefault(name, {"calls": 0, "interval_count": 0,
                                            "interval_sum_ns": 0, "max_duration_ns": 0})
        row["calls"] += 1
        if previous is not None:
            interval = now_ns - previous
            row["interval_count"] += 1
            row["interval_sum_ns"] += interval
            row["last_interval_ns"] = interval
            row["min_interval_ns"] = min(row.get("min_interval_ns", interval), interval)
            if interval >= row.get("max_interval_ns", -1):
                row.update(max_interval_ns=interval, max_interval_start_ns=previous,
                           max_interval_end_ns=now_ns)
        return now_ns

    def finish(self, name, started_ns, now_ns):
        row = self._timing[name]
        duration = now_ns - started_ns
        row["last_duration_ns"] = duration
        if duration >= row["max_duration_ns"]:
            row.update(max_duration_ns=duration, max_duration_start_ns=started_ns,
                       max_duration_end_ns=now_ns)

    def event(self, event):
        event = dict(event)
        self.counts[event["event"]] += 1
        # Only repeated adjacent identical errors are summarized, retaining exact
        # first/last timestamps and count. ARM/STOP/watchdog edges are never merged.
        if (event["event"] in ("host_error", "drive_rejected") and self._events
                and self._events[-1]["event"] == event["event"]
                and self._events[-1].get("error") == event.get("error")
                and self._events[-1].get("reason") == event.get("reason")):
            last = self._events[-1]
            last["repeats"] = last.get("repeats", 1) + 1
            last["last_monotonic_ms"] = event["monotonic_ms"]
            last["last_ros_ns"] = event.get("ros_ns")
            return
        self._sequence += 1
        event["sequence"] = self._sequence
        if len(self._events) == self._events.maxlen:
            self.dropped_events += 1
        self._events.append(event)

    def drain(self):
        timing = self._timing
        for row in timing.values():
            row["mean_interval_ns"] = (row["interval_sum_ns"] / row["interval_count"]
                                       if row["interval_count"] else None)
        result = {"timers": timing, "events": list(self._events),
                  "event_counts": dict(self.counts), "dropped_events": self.dropped_events}
        self._timing = {}
        self._events.clear()
        return result

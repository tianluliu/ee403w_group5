"""Observational exploration metrics; no policy or ROS dependencies."""

import json
import math
from collections import Counter


class Benchmark:
    def __init__(self, now, metadata):
        self.created = now
        self.started = None
        self.finished = None
        self.distance = 0.0
        self.previous = None
        self.last_stamp = None
        self.max_gap = 0.0
        self.samples = 0
        self.counts = Counter()
        self.failures = []
        self.outcome = None
        self.metadata = metadata

    def event(self, name, now, **fields):
        if self.finished is not None:
            return None
        if name == "ready" and self.started is None:
            self.started = now
        self.counts[name] += 1
        if name in ("navigation_result", "planner_result"):
            self.counts[f"{name}_status_{fields['status']}"] += 1
        if name == "failure":
            self.counts[f"failure_{fields['phase']}"] += 1
        if name == "suppression":
            self.failures.append((fields["x"], fields["y"], fields["expires"]))
        if name == "latched":
            if any(expiry <= now and math.hypot(fields["x"] - x, fields["y"] - y)
                   <= self.metadata["suppression_radius_m"] for x, y, expiry in self.failures):
                self.counts["retry"] += 1
        if name in ("complete", "interrupted", "timeout", "error"):
            self.finished = now
            self.outcome = name
        return {"schema_version": 1, "event": name, "elapsed_s": now - self.created, **fields}

    def odometry(self, x, y, stamp):
        if self.started is None or self.finished is not None:
            return
        if not all(math.isfinite(v) for v in (x, y, stamp)):
            self.counts["invalid_odometry"] += 1
            return
        if self.last_stamp is not None:
            if stamp <= self.last_stamp:
                self.counts["nonincreasing_odometry"] += 1
                return
            self.max_gap = max(self.max_gap, stamp - self.last_stamp)
            self.distance += math.hypot(x - self.previous[0], y - self.previous[1])
        self.previous = (x, y)
        self.last_stamp = stamp
        self.samples += 1

    def summary(self, now, state):
        end = self.finished if self.finished is not None else now
        return {"schema_version": 1, "metadata": self.metadata, "state": state, "outcome": self.outcome,
                "elapsed_s": None if self.started is None else end - self.started,
                "startup_s": None if self.started is None else self.started - self.created,
                "path_distance_m": self.distance, "odometry_samples": self.samples,
                "max_odometry_gap_s": self.max_gap, "counts": dict(self.counts),
                "coverage": None,
                "coverage_unavailable_reason": "No aligned ground-truth grid supplied"}


class BenchmarkLog:
    """Exclusive output creation; callers isolate I/O errors from control callbacks."""

    def __init__(self, path, now, metadata):
        self.metrics = Benchmark(now, metadata)
        self.stream = open(path, "x", buffering=1, encoding="utf-8")
        self.stream.write(json.dumps({"schema_version": 1, "event": "metadata", **metadata}) + "\n")

    def event(self, name, now, **fields):
        row = self.metrics.event(name, now, **fields)
        if row is not None:
            self.stream.write(json.dumps(row, allow_nan=False) + "\n")

    def close(self, now, state):
        self.stream.write(json.dumps({"event": "summary", **self.metrics.summary(now, state)},
                                     allow_nan=False) + "\n")
        self.stream.close()

"""ROS-independent measurements and hard gates for the sim-05 replay experiment."""
import hashlib
import json
import math
from pathlib import Path

EVENT_NS = 1793829384677301288
EDGES = {'/tf': ('odom', 'base_link'), '/tf_static': ('base_link', 'laser')}


def retained_transforms(topic, transforms):
    return [t for t in transforms
            if (t.header.frame_id, t.child_frame_id) == EDGES[topic]]


def digest_tree(path):
    root = Path(path)
    h = hashlib.sha256()
    for p in sorted(root.rglob('*')):
        if p.is_file():
            h.update(str(p.relative_to(root)).encode())
            with p.open('rb') as f:
                for block in iter(lambda: f.read(1024 * 1024), b''):
                    h.update(block)
    return h.hexdigest()


def parameter_changes(a, b, prefix=''):
    if isinstance(a, dict) and isinstance(b, dict):
        return [x for k in sorted(a.keys() | b.keys())
                for x in parameter_changes(a.get(k), b.get(k), prefix + '/' + k)]
    if isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        return [x for i, (left, right) in enumerate(zip(a, b))
                for x in parameter_changes(left, right, prefix + '/' + str(i))]
    return [] if a == b and type(a) is type(b) else [prefix]


def check_ab(a, b):
    changes = parameter_changes(a, b)
    if changes != ['/do_loop_closing'] or a.get('do_loop_closing') is not True or b.get('do_loop_closing') is not False:
        raise ValueError(f'A/B parameters differ unexpectedly: {changes}')
    if a.get('use_sim_time') is not True or a.get('restamp_tf') is not True:
        raise ValueError('Replay requires use_sim_time and restamp_tf')


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def wall_distance(x, y, segments):
    def distance(s):
        ax, ay, bx, by = s
        dx, dy = bx-ax, by-ay
        t = max(0., min(1., ((x-ax)*dx+(y-ay)*dy)/(dx*dx+dy*dy))) if dx or dy else 0.
        return math.hypot(x-ax-t*dx, y-ay-t*dy)
    return min(map(distance, segments))


def quantile(values, fraction):
    if not values:
        return None
    index = (len(values)-1)*fraction
    lo = int(index)
    return values[lo] + (values[min(lo+1, len(values)-1)]-values[lo])*(index-lo)


def grid_metrics(data, width, height, resolution, origin, segments):
    """Cell centers, >=65 occupied; fixed world alignment (no fitted registration)."""
    ox, oy, theta = origin
    c, s = math.cos(theta), math.sin(theta)
    distances = []
    for i, value in enumerate(data):
        if value >= 65:
            x, y = (i % width + .5)*resolution, (i // width + .5)*resolution
            distances.append(wall_distance(ox+c*x-s*y, oy+s*x+c*y, segments))
    distances.sort()
    return dict(width=width, height=height, resolution=resolution, origin=origin,
                occupied=len(distances), median_m=quantile(distances, .5),
                p90_m=quantile(distances, .9),
                fraction_over_015=sum(d > .15 for d in distances)/len(distances) if distances else None)


def reproduced(report):
    """Predefined conjunction: >1 m correction near original event AND grid damage."""
    maps = report['maps']
    for jump in report['jumps']:
        t = jump['time_ns']
        if jump['translation_m'] <= 1 or abs(t-EVENT_NS) > 2_000_000_000:
            continue
        before = [m for m in maps if m['time_ns'] < t and m['fraction_over_015'] is not None]
        after = [m for m in maps if t <= m['time_ns'] <= t+2_000_000_000 and m['fraction_over_015'] is not None]
        if before and any(m['fraction_over_015'] >= .25 and
                          m['fraction_over_015']-before[-1]['fraction_over_015'] >= .20 for m in after):
            return True
    return False


def baseline_gate(reports):
    if len(reports) != 2 or [r['trial'] for r in reports] != ['A1', 'A2']:
        raise ValueError('Exactly A1 and A2 required')
    for r in reports:
        if not r['complete'] or not reproduced(r):
            raise ValueError(f"{r['trial']} did not reproduce; STOP, no B")
        if any(r['parameters'].get(k) is not True for k in
               ('do_loop_closing', 'restamp_tf', 'use_sim_time')):
            raise ValueError('Baseline requires loop closure, restamping and bag time')
    for key in ('parameters', 'input_digest', 'versions', 'settings'):
        if reports[0][key] != reports[1][key]:
            raise ValueError(f'Baseline provenance mismatch: {key}')
    return {'passed': True, 'report_hashes': [hashlib.sha256(json.dumps(r, sort_keys=True).encode()).hexdigest() for r in reports]}

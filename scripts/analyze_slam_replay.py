#!/usr/bin/env python3
"""Measure raw replay maps and TF, evaluate A1/A2 gate, or compare A/B reports."""
import argparse
import json
from pathlib import Path
import re
from prepare_slam_replay import messages, require_pi
from lidar_robot.slam_replay import (angle_delta, baseline_gate, check_ab, grid_metrics,
                                     EVENT_NS, reproduced, yaw)
import math


def analyze(prepared, run):
    require_pi()
    manifest = json.loads((prepared/'manifest.json').read_text())
    report = json.loads((run/'run.json').read_text())
    maps, jumps, logs, previous = [], [], [], None
    for topic, msg, stamp in messages(run/'bag'):
        if topic == '/map':
            if msg.header.frame_id != 'map':
                raise ValueError('Unexpected map frame')
            o = msg.info.origin
            metric = grid_metrics(msg.data, msg.info.width, msg.info.height, msg.info.resolution,
                                  [o.position.x, o.position.y, yaw(o.orientation)], manifest['world_segments'])
            metric.update(time_ns=stamp, header_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
            maps.append(metric)
        elif topic == '/tf':
            for t in msg.transforms:
                if (t.header.frame_id, t.child_frame_id) == ('map', 'odom'):
                    current = [t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation)]
                    if previous:
                        jumps.append(dict(time_ns=stamp,
                            header_ns=t.header.stamp.sec*10**9+t.header.stamp.nanosec,
                            before=previous, after=current,
                            translation_m=math.hypot(current[0]-previous[0], current[1]-previous[1]),
                            yaw_rad=abs(angle_delta(current[2], previous[2]))))
                    previous = current
        elif topic == '/rosout' and (msg.name == 'slam_toolbox' or re.search('slam|ceres', msg.name, re.I)):
            logs.append(dict(time_ns=stamp, level=msg.level, text=msg.msg))
    if not maps or not jumps:
        raise ValueError('Missing replay map/TF output; cannot evaluate experiment')
    report.update(maps=maps, jumps=jumps, slam_logs=logs,
                  largest_translation_jump=max(jumps, key=lambda j: j['translation_m']),
                  largest_yaw_jump=max(jumps, key=lambda j: j['yaw_rad']),
                  first_major_distortion_ns=next((m['time_ns'] for m in maps
                      if m['fraction_over_015'] is not None and m['fraction_over_015'] >= .25), None),
                  missing_prebag_graph=True, fixed_alignment=manifest['alignment'])
    report['reproduced'] = reproduced(report)
    near_original = [j for j in jumps if abs(j['time_ns']-EVENT_NS) <= 2_000_000_000]
    report['original_event_comparison'] = dict(
        expected_translation_m=1.5203791782534, original_time_ns=EVENT_NS,
        largest_near_sequence=max(near_original, key=lambda j: j['translation_m']) if near_original else None,
        abrupt_correction_and_map_damage_reproduced=report['reproduced'])
    pattern = re.compile(r'loop|optim|ceres|dropp|queue|filter|extrapolat', re.I)
    report['console_diagnostics'] = [line for line in (run/'slam-console.log').read_text().splitlines() if pattern.search(line)]
    (run/'analysis.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ('trial', 'reproduced', 'largest_translation_jump',
                                             'largest_yaw_jump', 'first_major_distortion_ns')}, indent=2))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    a = sub.add_parser('analyze')
    a.add_argument('--prepared', required=True, type=Path)
    a.add_argument('--run', required=True, type=Path)
    g = sub.add_parser('gate')
    g.add_argument('--runs', required=True, type=Path)
    c = sub.add_parser('compare')
    c.add_argument('--runs', required=True, type=Path)
    args = p.parse_args()
    if args.command == 'analyze':
        analyze(args.prepared, args.run)
        return
    reports = [json.loads((args.runs/t/'analysis.json').read_text()) for t in ('A1', 'A2')]
    gate = baseline_gate(reports)
    if args.command == 'gate':
        (args.runs/'gate.json').write_text(json.dumps(gate, indent=2)+'\n')
        print('PASS: A1 and A2 reproduced. B may now be attempted.')
    else:
        for trial in ('B1', 'B2'):
            b = json.loads((args.runs/trial/'analysis.json').read_text())
            check_ab(reports[0]['parameters'], b['parameters'])
            for key in ('input_digest', 'versions', 'settings'):
                if reports[0][key] != b[key]:
                    raise ValueError(f'Comparison mismatch: {key}')
            if not b['complete']:
                raise ValueError('Incomplete B trial')
            reports.append(b)
        comparison = [{k: r[k] for k in ('trial', 'reproduced', 'largest_translation_jump',
                        'largest_yaw_jump', 'first_major_distortion_ns', 'maps')} for r in reports]
        (args.runs/'comparison.json').write_text(json.dumps(comparison, indent=2)+'\n')
        print('Wrote comparison.json; inspect map evolution and logs before causal conclusions.')


if __name__ == '__main__':
    main()

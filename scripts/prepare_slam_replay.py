#!/usr/bin/env python3
"""Prepare an isolated sim-05 input bag. Requires Raspberry Pi / ROS 2 Jazzy."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from lidar_robot.slam_replay import digest_tree, retained_transforms


def require_pi():
    model = Path('/proc/device-tree/model')
    if platform.system() != 'Linux' or os.environ.get('ROS_DISTRO') != 'jazzy' or not model.exists() or 'Raspberry Pi' not in model.read_text():
        raise RuntimeError('Replay runtime requires Raspberry Pi with sourced ROS 2 Jazzy')


def reader(path):
    import rosbag2_py as rb
    r = rb.SequentialReader()
    r.open(rb.StorageOptions(uri=str(path), storage_id='mcap'), rb.ConverterOptions('', ''))
    return r


def messages(path):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = reader(path)
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    while r.has_next():
        topic, raw, stamp = r.read_next()
        yield topic, deserialize_message(raw, types[topic]), stamp


def sequence_digest(path):
    r = reader(path)
    h = hashlib.sha256()
    while r.has_next():
        topic, raw, stamp = r.read_next()
        h.update(topic.encode()+b'\0'+stamp.to_bytes(8, 'big')+len(raw).to_bytes(8, 'big')+raw)
    return h.hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--config', type=Path, default=Path('config/slam_toolbox_sim.yaml'))
    a = p.parse_args()
    require_pi()
    import rosbag2_py as rb
    import yaml
    from rclpy.serialization import deserialize_message, serialize_message
    from tf2_msgs.msg import TFMessage
    if a.output.resolve().is_relative_to(a.source.resolve()) or a.source.resolve().is_relative_to(a.output.resolve()):
        raise ValueError('Source and output must be disjoint')
    a.output.mkdir(parents=True, exist_ok=False)
    original = digest_tree(a.source)
    params = yaml.safe_load(a.config.read_text())['slam_toolbox']['ros__parameters']
    if params.get('restamp_tf') is not True or params.get('do_loop_closing') is not True:
        raise ValueError('Expected tested A configuration')
    (a.output/'source-config.yaml').write_text(a.config.read_text())
    params['use_sim_time'] = True
    (a.output/'A.yaml').write_text(yaml.safe_dump({'slam_toolbox': {'ros__parameters': params}}))
    params = dict(params, do_loop_closing=False)
    (a.output/'B.yaml').write_text(yaml.safe_dump({'slam_toolbox': {'ros__parameters': params}}))
    r = reader(a.source)
    w = rb.SequentialWriter()
    w.open(rb.StorageOptions(uri=str(a.output/'input'), storage_id='mcap'), rb.ConverterOptions('', ''))
    keep = {'/scan', '/odom', '/tf', '/tf_static'}
    for meta in r.get_all_topics_and_types():
        if meta.name in keep:
            w.create_topic(meta)
    counts, segments, start, end = {}, None, None, None
    expected = hashlib.sha256()
    while r.has_next():
        topic, raw, stamp = r.read_next()
        if topic == '/sim/world':
            from visualization_msgs.msg import MarkerArray
            world = deserialize_message(raw, MarkerArray)
            walls = next(m for m in world.markers if m.ns == 'sim_world')
            if walls.header.frame_id != 'odom' or walls.pose.position.x or walls.pose.position.y or walls.pose.orientation.w != 1.:
                raise ValueError('Unexpected world frame/pose')
            current = [[p.x, p.y, q.x, q.y] for p, q in zip(walls.points[::2], walls.points[1::2])]
            if segments is not None and current != segments:
                raise ValueError('World changed')
            segments = current
        if topic not in keep:
            continue
        if topic in ('/tf', '/tf_static'):
            msg = deserialize_message(raw, TFMessage)
            msg.transforms = retained_transforms(topic, msg.transforms)
            if not msg.transforms:
                continue
            raw = serialize_message(msg)
        expected.update(topic.encode()+b'\0'+stamp.to_bytes(8, 'big')+len(raw).to_bytes(8, 'big')+raw)
        w.write(topic, raw, stamp)
        start = stamp if start is None else start
        end = stamp
        counts[topic] = counts.get(topic, 0)+1
    del w
    if set(counts) != keep or not segments or original != digest_tree(a.source):
        raise ValueError('Incomplete input or source changed')
    actual_digest = sequence_digest(a.output/'input')
    if actual_digest != expected.hexdigest():
        raise ValueError('Written input changed payloads, timestamps or ordering')
    manifest = dict(source=str(a.source.resolve()), source_digest=original,
                    input_digest=actual_digest, counts=counts,
                    start_ns=start, end_ns=end, world_segments=segments,
                    alignment=[0., 0., 0.], missing_prebag_graph=True)
    (a.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()

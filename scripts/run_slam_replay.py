#!/usr/bin/env python3
"""Run one fresh, isolated Pi/Jazzy SLAM trial; never launch the live stack."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from prepare_slam_replay import require_pi, sequence_digest
from lidar_robot.slam_replay import baseline_gate, check_ab, parameter_changes

SETTINGS = dict(rate=1., clock_hz=100, read_ahead=1000, drain_wall_seconds=5,
                qos='recorded input metadata; recorder explicit transient-local static TF')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepared', required=True, type=Path)
    p.add_argument('--runs', required=True, type=Path)
    p.add_argument('--trial', required=True, choices=['A1', 'A2', 'B1', 'B2'])
    p.add_argument('--domain', type=int, default=83)
    a = p.parse_args()
    require_pi()
    import yaml
    import rclpy
    from lifecycle_msgs.srv import ChangeState
    from rcl_interfaces.srv import ListParameters, GetParameters
    from rosbag2_interfaces.srv import Resume
    from rclpy.parameter import parameter_value_to_python
    if not 1 <= a.domain <= 232:
        raise ValueError('Use an unused domain 1..232')
    manifest = json.loads((a.prepared/'manifest.json').read_text())
    if sequence_digest(a.prepared/'input') != manifest['input_digest']:
        raise ValueError('Filtered input modified')
    baseline = None
    if a.trial.startswith('B'):
        baseline = [json.loads((a.runs/t/'analysis.json').read_text()) for t in ('A1', 'A2')]
        baseline_gate(baseline)  # Recompute, never trust an editable gate flag.
    output = a.runs/a.trial
    output.mkdir(parents=True, exist_ok=False)
    config = yaml.safe_load((a.prepared/(a.trial[0]+'.yaml')).read_text())
    requested = config['slam_toolbox']['ros__parameters']
    reference = yaml.safe_load((a.prepared/'source-config.yaml').read_text())['slam_toolbox']['ros__parameters']
    reference['use_sim_time'] = True
    if a.trial.startswith('B'):
        check_ab(reference, requested)
    elif parameter_changes(reference, requested):
        raise ValueError('A configuration changed')
    (output/'parameters.yaml').write_text(yaml.safe_dump(config))
    versions = subprocess.check_output(['dpkg-query', '-W', '-f=${Package}=${Version}\n',
        'ros-jazzy-*'], text=True)
    harness_hash = hashlib.sha256()
    for path in [Path(__file__), Path(__file__).with_name('prepare_slam_replay.py'),
                 Path(__file__).with_name('analyze_slam_replay.py'),
                 Path(__file__).resolve().parents[1]/'src/lidar_robot/slam_replay.py']:
        harness_hash.update(path.read_bytes())
    replay_environment = {key: os.environ.get(key) for key in (
        'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE',
        'FASTDDS_DEFAULT_PROFILES_FILE', 'ROS_STATIC_PEERS', 'ROS_LOCALHOST_ONLY')}
    settings = dict(SETTINGS, domain=a.domain, harness_digest=harness_hash.hexdigest(),
                    middleware_environment=replay_environment)
    if baseline and (baseline[0]['versions'] != versions or baseline[0]['settings'] != settings or
                     baseline[0]['input_digest'] != manifest['input_digest']):
        raise ValueError('A/B provenance differs')
    os.environ['ROS_DOMAIN_ID'] = str(a.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    # Direct DDS graph APIs, not the ros2 daemon (which may use a different domain).
    rclpy.init()
    node = rclpy.create_node('slam_replay_supervisor')
    processes, streams = [], []
    record = dict(trial=a.trial, complete=False, parameters={}, versions=versions,
                  settings=settings, input_digest=manifest['input_digest'])

    def wait(predicate, timeout=30):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if any(proc.poll() is not None for proc in processes):
                raise RuntimeError('Replay child exited before readiness/completion')
            result = predicate()
            if result:
                return result
        raise TimeoutError('Replay readiness timeout')

    def call(kind, name, request):
        client = node.create_client(kind, name)
        if not client.wait_for_service(timeout_sec=30):
            raise TimeoutError(name)
        future = client.call_async(request)
        wait(future.done)
        result = future.result()
        node.destroy_client(client)
        return result

    def spawn(command, filename):
        stream = (output/filename).open('w')
        streams.append(stream)
        proc = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc)
        return proc

    try:
        # Discovery grace period before asserting the domain is empty.
        until = time.monotonic()+3
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
        if set(node.get_node_names()) != {'slam_replay_supervisor'}:
            raise RuntimeError('Domain is not empty; stop other nodes or choose another domain')
        spawn(['ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node', '--ros-args',
               '--params-file', str(output/'parameters.yaml')], 'slam-console.log')
        for transition in (1, 3):  # configure, activate
            req = ChangeState.Request()
            req.transition.id = transition
            if not call(ChangeState, '/slam_toolbox/change_state', req).success:
                raise RuntimeError('SLAM lifecycle transition failed')
        names = call(ListParameters, '/slam_toolbox/list_parameters', ListParameters.Request()).result.names
        req = GetParameters.Request(names=names)
        values = call(GetParameters, '/slam_toolbox/get_parameters', req).values
        effective = json.loads(json.dumps(dict(zip(names, map(parameter_value_to_python, values))), default=list))
        record['parameters'] = effective
        (output/'effective-parameters.json').write_text(json.dumps(effective, indent=2)+'\n')
        for k, value in requested.items():
            if effective.get(k) != value:
                raise ValueError(f'Effective parameter mismatch: {k}')
        if baseline:
            check_ab(baseline[0]['parameters'], effective)
            check_ab(baseline[1]['parameters'], effective)
        # Ensure the static transform reaches the recorder even if sent before discovery.
        qos = {'/tf_static': {'reliability': 'reliable', 'durability': 'transient_local',
                              'history': 'keep_last', 'depth': 1}}
        (output/'record-qos.yaml').write_text(yaml.safe_dump(qos))
        spawn(['ros2', 'bag', 'record', '--use-sim-time', '--qos-profile-overrides-path',
               str(output/'record-qos.yaml'), '-s', 'mcap', '-o', str(output/'bag'),
               '/clock', '/map', '/tf', '/tf_static', '/rosout', '/pose', '/slam_toolbox/graph_visualization'],
              'recorder-console.log')
        wait(lambda: any(e.node_name.startswith('rosbag2_recorder')
                         for e in node.get_subscriptions_info_by_topic('/tf_static')) and
                     any(e.node_name.startswith('rosbag2_recorder')
                         for e in node.get_subscriptions_info_by_topic('/map')))
        player = spawn(['ros2', 'bag', 'play', str(a.prepared/'input'), '--clock', '100',
                        '--rate', '1.0', '--start-paused', '--disable-keyboard-controls',
                        '--read-ahead-queue-size', '1000'], 'player-console.log')
        service = wait(lambda: next((name for name, types in node.get_service_names_and_types()
                                     if 'rosbag2_interfaces/srv/Resume' in types), None))
        wait(lambda: node.count_publishers('/scan') == 1 and node.count_publishers('/odom') == 1)
        # Snapshot authorities; input publishers must all belong to the player.
        authorities = {topic: [e.node_name for e in node.get_publishers_info_by_topic(topic)]
                       for topic in ('/scan', '/odom', '/tf', '/tf_static')}
        player_names = set(authorities['/scan'])
        if set(authorities['/odom']) != player_names or any(
                owner not in player_names | {'slam_toolbox'} for owners in authorities.values() for owner in owners):
            raise RuntimeError('Unexpected TF/input authority')
        record['authorities'] = authorities
        call(Resume, service, Resume.Request())
        deadline = time.monotonic()+(manifest['end_ns']-manifest['start_ns'])/1e9+120
        while player.poll() is None:
            rclpy.spin_once(node, timeout_sec=.1)
            if any(proc.poll() is not None for proc in processes if proc is not player):
                raise RuntimeError('SLAM/recorder exited during playback')
            if time.monotonic() > deadline:
                raise TimeoutError('Playback exceeded duration budget')
        if player.returncode:
            raise RuntimeError('Playback failed')
        time.sleep(SETTINGS['drain_wall_seconds'])
        if any(proc.poll() is not None for proc in processes if proc is not player):
            raise RuntimeError('SLAM/recorder exited while draining')
        record['complete'] = True
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
                    record['complete'] = False
        for stream in streams:
            stream.close()
        (output/'run.json').write_text(json.dumps(record, indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()
    subprocess.run([sys.executable, str(Path(__file__).with_name('analyze_slam_replay.py')),
                    'analyze', '--prepared', str(a.prepared), '--run', str(output)], check=True)


if __name__ == '__main__':
    main()

"""Save a live Cartographer state and a ROS occupancy-grid snapshot."""
import argparse
from pathlib import Path
import subprocess
import rclpy
from cartographer_ros_msgs.srv import WriteState


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix', type=Path, help='Output prefix, e.g. maps/my_maze')
    parser.add_argument('--overwrite', action='store_true')
    parser.add_argument('--real-time', action='store_true', help='Use wall time for map saver on real hardware')
    options = parser.parse_args(args)
    prefix = options.prefix.expanduser().resolve()
    targets = [prefix.with_suffix(ext) for ext in ('.pbstream', '.yaml', '.pgm')]
    # A prefix with a dot is ambiguous when the CLI appends suffixes.
    if prefix.suffix:
        parser.error('Pass a prefix without a filename extension')
    if not options.overwrite and any(p.exists() for p in targets):
        parser.error('Output already exists; choose another prefix or pass --overwrite')
    prefix.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[])
    node = rclpy.create_node('save_carto_map')
    try:
        client = node.create_client(WriteState, '/write_state')
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('/write_state unavailable: start Cartographer first')
        request = WriteState.Request()
        request.filename = str(targets[0])
        request.include_unfinished_submaps = True
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=60)
        if not future.done():
            raise RuntimeError('Cartographer write_state timed out')
        result = future.result()
        if result.status.code != 0:
            raise RuntimeError(result.status.message)
        print(f'Saved Cartographer state: {targets[0]}', flush=True)
        subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(prefix),
                        '--ros-args', '-p', f'use_sim_time:={str(not options.real_time).lower()}',
                        '-p', 'save_map_timeout:=10.0'], check=True, timeout=30)
        print(f'Saved occupancy map: {targets[1]}\nThis is the area observed so far; mapping can continue.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

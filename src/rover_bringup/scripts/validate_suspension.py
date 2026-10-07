#!/usr/bin/env python3
"""Explicit simulation-only controlled traverse; emits a JSON measurement log.

Start the headless terrain world with joystick:=false. Use --drive to authorize
synthetic R2 input. This is not a hardware controller/test script.
"""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState, Joy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive',action='store_true')
    parser.add_argument('--seconds',type=float,default=15.)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('suspension_validation')
    node.set_parameters([rclpy.Parameter('use_sim_time',value=True)])
    now=[None]; state={}; odom=[None]; records=[]
    simulation=[False]
    node.create_subscription(String,'/robot_description',
        lambda m:simulation.__setitem__(0,'librover_differential.so' in m.data),
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Clock,'/clock',lambda m:now.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
    def joints(m):state.update(zip(m.name,m.position))
    node.create_subscription(JointState,'/joint_states',joints,10)
    node.create_subscription(Odometry,'/odom',lambda m:odom.__setitem__(0,m),10)
    joy=node.create_publisher(Joy,'/joy',10)
    wall=time.monotonic();start=None;last_send=0.;last_record=-1.
    try:
        while time.monotonic()-wall<max(60.,args.seconds*5):
            rclpy.spin_once(node,timeout_sec=.01)
            if now[0] is None or not simulation[0]:continue
            if start is None:start=now[0]
            elapsed=now[0]-start
            if elapsed>args.seconds:break
            if time.monotonic()-last_send>.05:
                msg=Joy();msg.axes=[0.]*6;msg.buttons=[0]*15
                # Neutral for first/last two simulation seconds. Half-pressed
                # R2 yields ~0.274 m/s with the configured deadzone.
                if args.drive and 2.<elapsed<args.seconds-2.:msg.axes[5]=-.5
                joy.publish(msg);last_send=time.monotonic()
            if elapsed-last_record>.1 and odom[0] is not None and 'left_rocker_joint' in state:
                p=odom[0].pose.pose.position;q=odom[0].pose.pose.orientation
                sample={'t':elapsed,'position':[p.x,p.y,p.z],
                        'pitch':math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x)))),
                        'joints':{k:v for k,v in state.items() if 'rocker' in k or 'bogie' in k}}
                records.append(sample);last_record=elapsed
    finally:
        # Circle stop + neutral; watchdog is a second layer on interruption.
        msg=Joy();msg.axes=[0.]*6;msg.buttons=[0,1]+[0]*13
        if simulation[0]:
            for _ in range(5):joy.publish(msg);rclpy.spin_once(node,timeout_sec=.02)
        node.destroy_node();rclpy.shutdown()
    if not records:raise RuntimeError('No valid simulation joint/odometry samples')
    names=records[0]['joints']
    summary={'drive':args.drive,'duration_sim':records[-1]['t'],
        'duration_wall':time.monotonic()-wall,
        'joint_ranges':{k:[min(r['joints'][k] for r in records),max(r['joints'][k] for r in records)] for k in names},
        'max_differential_error_rad':max(abs(r['joints']['left_rocker_joint']+r['joints']['right_rocker_joint']) for r in records),
        'start_position':records[0]['position'],'end_position':records[-1]['position'],
        'height_range':[min(r['position'][2] for r in records),max(r['position'][2] for r in records)],
        'pitch_range':[min(r['pitch'] for r in records),max(r['pitch'] for r in records)]}
    args.output.write_text(json.dumps({'summary':summary,'samples':records},indent=2)+'\n')
    print(json.dumps(summary,indent=2))
    assert summary['duration_sim'] >= args.seconds-.5, 'Timed out before finishing requested simulation duration'
    assert all(math.isfinite(v) for r in records for v in r['joints'].values())
    assert summary['max_differential_error_rad'] < .1, 'Differential is not constraining rocker sum'
    assert all(abs(r['joints'][k])<(.61 if 'rocker' in k else .76) for r in records for k in names)


if __name__=='__main__':main()

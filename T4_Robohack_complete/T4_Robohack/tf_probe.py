#!/usr/bin/env python3
"""Read-only A/B test of TF reception; no motion commands."""
import argparse
import json
import time
from collections import Counter
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf_receive import body_transforms


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--seconds',type=float,default=20.)
    parser.add_argument('--mode', choices=['fast','reliable','old','both'], default='fast')
    args=parser.parse_args()
    rclpy.init(); n=Node('t4_tf_reception_probe')
    samples={'odom':{},'tf_old':{},'tf_reliable':{}}
    traffic={'tf_old':Counter(),'tf_reliable':Counter()}
    def save(key,header,p,q):
        stamp=header.stamp.sec*1_000_000_000+header.stamp.nanosec
        samples[key][stamp]=(p.x,p.y,p.z,q.x,q.y,q.z,q.w)
    def tf_cb(msg,key):
        traffic[key]['messages']+=1
        traffic[key]['transforms']+=len(msg.transforms)
        for tr in msg.transforms:
            traffic[key]['frame '+tr.header.frame_id+' -> '+tr.child_frame_id]+=1
            if tr.header.frame_id=='odom' and tr.child_frame_id=='trunk':
                save(key,tr.header,tr.transform.translation,tr.transform.rotation)
    n.create_subscription(Odometry,'/odom',lambda m:save('odom',m.header,m.pose.pose.position,m.pose.pose.orientation),QoSProfile(depth=1000))
    if args.mode in ('old','both'):
        n.create_subscription(TFMessage,'/tf',lambda m:tf_cb(m,'tf_old'),qos_profile_sensor_data)
    if args.mode in ('reliable','both'):
        n.create_subscription(TFMessage,'/tf',lambda m:tf_cb(m,'tf_reliable'),QoSProfile(depth=10000))
    if args.mode == 'fast':
        def raw_tf(data):
            traffic['tf_reliable']['serialized_batches_received'] += 1
            selected = body_transforms(data)
            traffic['tf_reliable']['body_transforms_received'] += len(selected)
            for tr in selected:
                save('tf_reliable', tr.header, tr.transform.translation, tr.transform.rotation)
        n.create_subscription(TFMessage, '/tf', raw_tf, QoSProfile(depth=1000), raw=True)
    try:
        deadline=time.monotonic()+args.seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.01)
        report={}
        for key,poses in samples.items():
            stamps=sorted(poses)
            gaps=[(b-a)*1e-9 for a,b in zip(stamps,stamps[1:])]
            report[key]={'count':len(stamps),'mean_hz':(len(stamps)-1)/((stamps[-1]-stamps[0])*1e-9) if gaps else None,
                         'max_gap_s':max(gaps) if gaps else None,'gaps_over_50ms':sum(g>.050001 for g in gaps),
                         'first_stamp_s':stamps[0]*1e-9 if stamps else None,
                         'last_stamp_s':stamps[-1]*1e-9 if stamps else None}
        for key in ('tf_old','tf_reliable'):
            if not samples[key] or not samples['odom']:continue
            lo=max(min(samples[key]),min(samples['odom'])); hi=min(max(samples[key]),max(samples['odom']))
            expected={s for s in samples['odom'] if lo<=s<=hi}
            common=expected & samples[key].keys()
            report[key]['matched_fraction']=len(common)/len(expected) if expected else None
            report[key]['start_delay_vs_odom_s']=(min(samples[key])-min(samples['odom']))*1e-9
            report[key]['end_lag_vs_odom_s']=(max(samples['odom'])-max(samples[key]))*1e-9
            report[key]['full_odom_coverage']=len(samples['odom'].keys() & samples[key].keys())/len(samples['odom'])
            report[key]['pose_matches']=all(samples[key][s]==samples['odom'][s] for s in common) if common else None
        report['mode']=args.mode
        report['all_tf_traffic']={k:dict(v) for k,v in traffic.items()}
        report['tf_publishers']=[{'node':p.node_name,'namespace':p.node_namespace,'qos':str(p.qos_profile)} for p in n.get_publishers_info_by_topic('/tf')]
        print(json.dumps(report,indent=2))
    finally:
        n.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()

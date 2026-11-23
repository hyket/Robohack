#!/usr/bin/env python3
"""LOCAL evaluation only; never copied into the submission image.
With --drive publishes a square command sequence. Truth is used only for metrics.
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from metrics import load, evaluate
from tf_receive import body_transforms


def main():
    p=argparse.ArgumentParser(); p.add_argument('--drive',action='store_true')
    p.add_argument('--output',default='results'); p.add_argument('--wall-timeout',type=float,default=1800)
    p.add_argument('--side',type=float,default=1.1,help='Nominal square side in metres (speed 0.2 m/s)')
    args=p.parse_args()
    if not math.isfinite(args.side) or args.side <= 0:
        p.error('--side must be a finite positive number')
    if not args.drive:
        p.error('Use --drive to explicitly authorize local simulated motion; never run alongside Judge.')
    destination=Path(args.output)/time.strftime('%Y%m%d-%H%M%S'); destination.mkdir(parents=True)
    rclpy.init(); n=Node('t4_local_trial',parameter_overrides=[Parameter('use_sim_time',value=True)])
    streams={}; files=[]; counts={'odom':0,'truth':0,'tf':0}; latest={}; frame_errors=[0]; ready=[False]
    for name in counts:
        f=(destination/f'{name}.csv').open('w',newline=''); files.append(f)
        streams[name]=csv.writer(f); streams[name].writerow(['t','x','y','z','qx','qy','qz','qw'])
    def save(name,t,pos,q):
        if t<=latest.get(name,-1): return
        streams[name].writerow([t,pos.x,pos.y,pos.z,q.x,q.y,q.z,q.w])
        counts[name]+=1; latest[name]=t
    def odom(msg,name):
        if name=='odom' and (msg.header.frame_id!='odom' or msg.child_frame_id!='trunk'): frame_errors[0]+=1
        save(name,msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,msg.pose.pose.position,msg.pose.pose.orientation)
    n.create_subscription(Odometry,'/odom',lambda m:odom(m,'odom'),QoSProfile(depth=1000))
    n.create_subscription(Odometry,'/odom/ground_truth',lambda m:odom(m,'truth'),qos_profile_sensor_data)
    def tf(serialized):
        for tr in body_transforms(serialized):
            if tr.header.frame_id=='odom' and tr.child_frame_id=='trunk':
                save('tf',tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9,tr.transform.translation,tr.transform.rotation)
    # /tf also contains high-rate joint transforms; do not use the 5-message sensor queue.
    n.create_subscription(TFMessage,'/tf',tf,QoSProfile(depth=1000),raw=True)
    n.create_subscription(Bool,'/go1_ready',lambda m:ready.__setitem__(0,m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pub=n.create_publisher(Twist,'/cmd_vel',10)
    # Initial rest, configurable legs, turns, five-second settling + 11-second rest.
    phases=[(5.,0.,0.)]+[(duration,v,w) for _ in range(4) for duration,v,w in [(args.side/.2,.2,0.),(math.pi/2/.3,0.,.3)]]
    moving_end=sum(x[0] for x in phases); phases += [(16.,0.,0.)]
    started=None; last_command=-1.; deadline=time.monotonic()+args.wall_timeout
    success=False
    try:
        while rclpy.ok():
            rclpy.spin_once(n,timeout_sec=.01)
            if time.monotonic()>deadline: raise TimeoutError('Wall timeout: inspect sensor topics / readiness / clock')
            now=n.get_clock().now().nanoseconds*1e-9
            if started is None:
                if ready[0] and now>0 and all(counts[k]>5 for k in counts):
                    started=now; print('LOCAL square started; output:',destination,flush=True)
                else: continue
            if now<started: raise RuntimeError('Simulation time reset during trial')
            if now-last_command<.05: continue
            elapsed=now-started; boundary=0.; cmd=Twist(); finished=True
            for duration,v,w in phases:
                boundary+=duration
                if elapsed<boundary:
                    cmd.linear.x=v; cmd.angular.z=w; finished=False; break
            pub.publish(cmd); last_command=now
            if finished: success=True; break
    except KeyboardInterrupt:
        print('Interrupted; partial CSV files retained')
    finally:
        for _ in range(5):
            pub.publish(Twist()); rclpy.spin_once(n,timeout_sec=.02)
        for f in files:f.close()
        n.destroy_node(); rclpy.try_shutdown()
    if success:
        import numpy as np
        estimate,truth,transforms=load(destination/'odom.csv'),load(destination/'truth.csv'),load(destination/'tf.csv')
        report=evaluate(truth,estimate,started+moving_end+5.)
        report['nominal_side_m']=args.side
        report['commanded_duration_s']=sum(x[0] for x in phases)
        common,ei,ti=np.intersect1d(estimate[:,0],transforms[:,0],return_indices=True)
        report['tf_mean_hz']=float((len(transforms)-1)/(transforms[-1,0]-transforms[0,0]))
        report['tf_max_gap_s']=float(np.max(np.diff(transforms[:,0])))
        report['tf_gaps_over_50ms']=int(np.sum(np.diff(transforms[:,0])>.050001))
        lo=max(estimate[0,0],transforms[0,0]); hi=min(estimate[-1,0],transforms[-1,0])
        expected=int(np.sum((estimate[:,0]>=lo)&(estimate[:,0]<=hi)))
        report['matching_tf_fraction']=len(common)/expected if expected else None
        report['tf_pose_matches']=bool(len(common)>0 and np.allclose(estimate[ei,1:],transforms[ti,1:],atol=1e-8))
        report['frame_errors']=frame_errors[0]
        report['note']='Local estimate only. Official Judge and hidden scenarios were not run.'
        (destination/'metrics.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))

if __name__=='__main__': main()

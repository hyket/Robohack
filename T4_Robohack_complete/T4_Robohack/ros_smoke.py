#!/usr/bin/env python3
"""Synthetic ROS contract test, use a dedicated ROS_DOMAIN_ID (e.g. 217)."""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from node import T4, seconds
from tf_receive import body_transforms


def main():
    rclpy.init(); sut=T4(); driver=Node('t4_test_driver')
    ex=SingleThreadedExecutor(); ex.add_node(sut); ex.add_node(driver)
    clock=driver.create_publisher(Clock,'/clock',10)
    imu=driver.create_publisher(Imu,'/imu/data',qos_profile_sensor_data)
    scan=driver.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
    od=[]; transforms={}
    driver.create_subscription(Odometry,'/odom',od.append,100)
    def tf_cb(serialized):
        for tf in body_transforms(serialized):
            if tf.header.frame_id=='odom' and tf.child_frame_id=='trunk':
                transforms[seconds(tf.header.stamp)]=tf
    driver.create_subscription(TFMessage,'/tf',tf_cb,100,raw=True)
    def pump(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end: ex.spin_once(timeout_sec=.001)
    try:
        pump(1.)
        angles=-math.pi+np.arange(720)*2*math.pi/720
        with np.errstate(divide='ignore'):
            x,y=np.cos(angles),np.sin(angles)
            ranges=np.minimum(np.where(x>0,3.78/x,-3.22/x),np.where(y>0,3/y,-3/y))
        for i in range(250):
            c=Clock(); c.clock.sec,c.clock.nanosec=divmod((i+1)*20_000_000,1_000_000_000)
            clock.publish(c)
            m=Imu(); m.header.stamp=c.clock; m.header.frame_id='imu_link'; m.orientation.w=1.; imu.publish(m)
            if i%3==0:
                s=LaserScan(); s.header.stamp=c.clock; s.header.frame_id='base_scan'
                s.angle_min=-math.pi; s.angle_increment=2*math.pi/720; s.range_min=.12; s.range_max=20.
                s.ranges=ranges.tolist(); scan.publish(s)
            pump(.02)
        pump(.2)
        assert len(od)>80, f'Only {len(od)} odometry messages'
        stamps=np.array([seconds(o.header.stamp) for o in od])
        assert np.all(np.diff(stamps)>0)
        hz=(len(stamps)-1)/(stamps[-1]-stamps[0]); assert hz>=20,hz
        for o in od:
            assert o.header.frame_id=='odom' and o.child_frame_id=='trunk'
            tf=transforms[seconds(o.header.stamp)]
            assert tf.transform.rotation==o.pose.pose.orientation
            assert tf.transform.translation.x==o.pose.pose.position.x
            assert tf.transform.translation.y==o.pose.pose.position.y
            assert tf.transform.translation.z==o.pose.pose.position.z
            q=o.pose.pose.orientation
            assert abs(np.linalg.norm([q.x,q.y,q.z,q.w])-1)<1e-6
        print(f'PASS: {len(od)} odom, {hz:.2f} Hz sim time, matching TF for every sample')
    finally:
        ex.shutdown(); sut.destroy_node(); driver.destroy_node(); rclpy.try_shutdown()

if __name__=='__main__': main()

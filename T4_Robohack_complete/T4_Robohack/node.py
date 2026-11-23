#!/usr/bin/env python3
"""T4 ROS adapter. Inputs only /imu/data, /scan, /clock."""
from collections import deque
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from core import Attitudes, Estimator, project


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class T4(Node):
    def __init__(self):
        super().__init__('t4_odometry', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.declare_parameter('laser_offset', [.22, 0., .11])
        self.offset = np.array(self.get_parameter('laser_offset').value)
        if self.offset.shape != (3,) or not np.isfinite(self.offset).all():
            raise ValueError('laser_offset must contain three finite values')
        self.reset()
        self.pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf = TransformBroadcaster(self)
        self.diag = self.create_publisher(String, '/t4/status', 10)
        self.create_subscription(Imu, '/imu/data', self.imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_timer(.02, self.publish)
        self.create_timer(1., self.status)
        self.get_logger().info('T4: lidar + IMU; /clock required; output 50 Hz simulation time')

    def reset(self):
        self.attitudes, self.estimator = Attitudes(), Estimator()
        self.pending = deque(maxlen=8)
        self.angular = np.zeros(3)
        self.last_clock = None
        self.last_published = None
        self.last_imu = None
        self.last_scan = None
        self.reason = 'waiting_for_sensors'
        self.rmse = 0.1

    def imu(self, msg):
        stamp = seconds(msg.header.stamp)
        if self.last_imu is not None and stamp < self.last_imu-.1:
            self.reset()
        if msg.header.frame_id.lstrip('/') != 'imu_link' or msg.orientation_covariance[0] < 0:
            self.reason = 'unsupported_imu_frame_or_orientation'
            return
        q = msg.orientation
        if self.attitudes.add(stamp, [q.x,q.y,q.z,q.w]):
            self.last_imu = stamp
            w = msg.angular_velocity
            self.angular = np.nan_to_num([w.x,w.y,w.z], nan=0., posinf=0., neginf=0.)
            self.process()

    def scan(self, msg):
        if msg.header.frame_id.lstrip('/') != 'base_scan':
            self.reason = 'unsupported_laser_frame'
            return
        stamp = seconds(msg.header.stamp)
        if self.last_scan is not None and stamp <= self.last_scan:
            return
        if not math.isfinite(msg.angle_increment) or msg.angle_increment <= 0:
            return
        self.last_scan = stamp
        self.pending.append(msg)
        self.process()

    def process(self):
        while self.pending and self.last_imu is not None:
            msg = self.pending[0]
            stamp = seconds(msg.header.stamp)
            if stamp > self.last_imu:
                return  # Wait for the IMU sample after this scan; then interpolate.
            self.pending.popleft()
            attitude = self.attitudes.at(stamp)
            if attitude is None:
                self.reason = 'missing_synchronized_imu'
                continue
            points, indices, yaw = project(msg.ranges, msg.angle_min, msg.angle_increment,
                msg.range_min, msg.range_max, attitude, self.offset)
            result = self.estimator.update(points, indices, yaw, stamp)
            self.reason = result.reason
            if result.accepted:
                self.rmse = result.rmse

    def publish(self):
        now = self.get_clock().now()
        t = now.nanoseconds*1e-9
        if self.last_clock is not None and t < self.last_clock:
            self.reset()
        self.last_clock = t
        if t <= 0 or self.estimator.reference is None or self.last_imu is None:
            return
        if self.last_published is not None and t <= self.last_published:
            return
        self.last_published = t
        rot = self.attitudes.at(self.last_imu)
        stale = t-self.estimator.last_stamp > .25 or t-self.last_imu > .1
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'trunk'
        odom.pose.pose.position.x, odom.pose.pose.position.y = map(float, self.estimator.position)
        q = rot.as_quat()
        o = odom.pose.pose.orientation
        o.x,o.y,o.z,o.w = map(float,q)
        uncertainty = 1. if stale else max(.0001, self.rmse**2)
        for idx,val in zip([0,7,14,21,28,35],[uncertainty,uncertainty,1.,.005,.005,.005]):
            odom.pose.covariance[idx] = val
        if not stale:
            v = rot.inv().apply([*self.estimator.velocity,0.])
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float,v)
            odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = map(float,self.angular)
        for idx in [0,7,14,21,28,35]:
            odom.twist.covariance[idx] = 1. if stale else .05
        tf = TransformStamped()
        tf.header, tf.child_frame_id = odom.header, odom.child_frame_id
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.rotation = o
        self.pub.publish(odom)
        self.tf.sendTransform(tf)

    def status(self):
        msg = String()
        msg.data = self.reason
        self.diag.publish(msg)
        if self.reason not in ('tracking','initialized'):
            self.get_logger().warning(self.reason)


def main():
    rclpy.init()
    node = T4()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

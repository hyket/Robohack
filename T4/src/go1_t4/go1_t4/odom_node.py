from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformBroadcaster

from .estimator import ScanTracker, project_scan


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class OdomNode(Node):
    def __init__(self):
        super().__init__('go1_t4_odometry')
        self.history = deque(maxlen=512)
        self.tracker = ScanTracker()
        self.last_scan = -1.0
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Imu, '/imu/data', self.imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_timer(1 / 40, self.publish)

    def imu(self, msg):
        q = msg.orientation
        quat = np.array([q.x, q.y, q.z, q.w])
        norm = np.linalg.norm(quat)
        if msg.orientation_covariance[0] == -1 or not np.isfinite(norm) or norm < 1e-8:
            return
        t = seconds(msg.header.stamp)
        if self.history and t <= self.history[-1][0]:
            return
        self.history.append((t, quat / norm))

    def scan(self, msg):
        t = seconds(msg.header.stamp)
        if not self.history or t <= self.last_scan:
            return
        sample, quat = min(self.history, key=lambda item: abs(item[0] - t))
        if abs(sample - t) > 0.075:
            return
        points = project_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                              msg.range_min, msg.range_max, quat)
        self.tracker.update(points)
        self.last_scan = t

    def publish(self):
        if not self.history:
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'trunk'
        pose = msg.pose.pose
        pose.position.x, pose.position.y = self.tracker.xy.tolist()
        q = pose.orientation
        q.x, q.y, q.z, q.w = self.history[-1][1].tolist()
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = msg.child_frame_id
        tf.transform.translation.x = pose.position.x
        tf.transform.translation.y = pose.position.y
        tf.transform.rotation = q
        self.odom.publish(msg)
        self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

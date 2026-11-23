"""Run after sourcing ROS. Tests real ROS CDR and measures parsing workload."""
import time
import unittest
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.serialization import serialize_message, deserialize_message
from tf_receive import body_transforms


def transform(parent,child):
    t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
    t.header.stamp.sec=123;t.header.stamp.nanosec=456
    t.transform.translation.x=1.234;t.transform.rotation.w=1.
    return t


class Tests(unittest.TestCase):
    def test_mixed_batch_preserves_pose_and_stamp(self):
        target=transform('odom','trunk')
        data=serialize_message(TFMessage(transforms=[transform('trunk','FL_hip'),target,transform('FL_hip','FL_thigh')]))
        self.assertEqual(body_transforms(data),(target,))

    def test_empty_and_joint_only(self):
        for transforms in [[],[transform('trunk','FL_hip')],[transform('some_odom','trunk')],[transform('odom','some_trunk')]]:
            self.assertEqual(body_transforms(serialize_message(TFMessage(transforms=transforms))),())

    def test_multiple_body_transforms_preserved(self):
        t=transform('odom','trunk')
        self.assertEqual(body_transforms(serialize_message(TFMessage(transforms=[t,t]))),(t,t))


def benchmark():
    joints=serialize_message(TFMessage(transforms=[transform('trunk',f'joint_{i}') for i in range(12)]))
    body=serialize_message(TFMessage(transforms=[transform('odom','trunk')]))
    batches=[joints]*20+[body]
    for name,receiver in [('decoded',lambda b:tuple(t for t in deserialize_message(b,TFMessage).transforms if t.header.frame_id=='odom' and t.child_frame_id=='trunk')),('raw_filter',body_transforms)]:
        started=time.perf_counter();count=0
        for _ in range(500):
            for b in batches:count+=len(receiver(b))
        print(name,'10500 batches; body count',count,'elapsed_s',round(time.perf_counter()-started,4))
        assert count==500

if __name__=='__main__':
    unittest.main(exit=False);benchmark()

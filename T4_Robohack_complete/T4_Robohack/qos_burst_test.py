import rclpy,time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
rclpy.init();n=Node('qos_burst_test');counts=[0,0]
def cb(m,i):
    counts[i]+=sum(t.child_frame_id=='trunk' for t in m.transforms)
subs=[n.create_subscription(TFMessage,'/test_tf',lambda m:cb(m,0),qos_profile_sensor_data),n.create_subscription(TFMessage,'/test_tf',lambda m:cb(m,1),QoSProfile(depth=1000))]
p=n.create_publisher(TFMessage,'/test_tf',1000)
def pump(dt):
    end=time.monotonic()+dt
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.001)
pump(1)
for _ in range(30):
    for j in range(21):
        t=TransformStamped();t.header.frame_id='odom';t.child_frame_id='trunk' if j==0 else 'joint'; t.transform.rotation.w=1.
        p.publish(TFMessage(transforms=[t]))
    # Model a slow reader or batched DDS delivery. Sleep only 20ms per burst.
    time.sleep(.02);pump(.04)
pump(.2)
print('Published trunk TF: 30; old best-effort depth5:',counts[0],'; reliable depth1000:',counts[1])
assert counts[1]==30
assert counts[0]<counts[1]
n.destroy_node();rclpy.shutdown()

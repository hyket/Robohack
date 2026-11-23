"""Skip irrelevant CDR TF batches before creating Python message objects.
The byte check is only a conservative prefilter. Every candidate is decoded by
ROS and checked by its actual frame identifiers. Never alters stamps or poses.
"""
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def body_transforms(serialized):
    # CDR encodes ROS strings as null-terminated UTF-8 in either endianness.
    # False positives (e.g. a joint named 'some_odom') are harmless: validate below.
    if b'odom\x00' not in serialized or b'trunk\x00' not in serialized:
        return ()
    message = deserialize_message(serialized, TFMessage)
    return tuple(tr for tr in message.transforms
                 if tr.header.frame_id == 'odom' and tr.child_frame_id == 'trunk')

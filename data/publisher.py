#! /usr/bin/env python

"""A rosnode in a subprocess that will publish input from stdin.

This takes in a pickled tuple of three values: ROSTOPIC, ROSMSG_TYPE, and
QUEUE_SIZE as its only command line argument. This means that it must be able
to unpickle the ROSMSG_TYPE argument, which requires importing std_msgs or
similar packages. So rosinit must have been done on this thread. This will have
been done by default, but might be a cause for odd issues.
"""

import cPickle as pickle
import sys
from StringIO import StringIO

import rospy


def publish(topic, msg_type):
    """A function to isolate the publishing logic.
    """
    rospy.init_node('mock_publish_'+topic.split('/')[-1], anonymous=True,
            disable_signals=True)
    pub = rospy.Publisher(topic, msg_type, queue_size=QUEUE_SIZE)
    msg = msg_type()
    sio = StringIO()
    msg.serialize(sio)
    while 1:
        data = sys.stdin.read(sio.len)
        msg.deserialize(data)
        pub.publish(msg)


if __name__ == '__main__':
    ROSTOPIC, ROSMSG_TYPE, QUEUE_SIZE = pickle.loads(sys.argv[1])
    publish(ROSTOPIC, ROSMSG_TYPE)

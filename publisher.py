#! /usr/bin/env python

"""A rosnode in a subprocess that will publish input from stdin.
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
    pub = rospy.Publisher(topic, msg_type, QUEUE_SIZE)
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

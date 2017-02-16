#! /usr/bin/env python

import cPickle as pickle
import sys
from StringIO import StringIO

import rospy


def publish(topic, msg_type):
    rospy.init_node('mock_publish_'+topic.split('/')[-1], anonymous=True,
            disable_signals=True)
    pub = rospy.Publisher(topic, msg_type, queue_size)
    msg = msg_type()
    s = StringIO()
    msg.serialize(s)
    while 1:
        data = sys.stdin.read(s.len)
        msg.deserialize(data)
        pub.publish(msg)
        

if __name__ == '__main__':
    rostopic, rosmsg_type, queue_size = pickle.loads(sys.argv[1])
    publish(rostopic, rosmsg_type)

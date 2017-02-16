#! /usr/bin/env python

import cPickle as pickle
import sys

import rospy


def listen(topic, msg_type):
    rospy.init_node('mock_listen_'+topic.split('/')[-1], anonymous=True,
            disable_signals=True)
    def listener_cb(message):
        message.serialize(sys.stdout)
        rospy.signal_shutdown('message sent')
    rospy.Subscriber(topic, msg_type, listener_cb)
    rospy.spin()



if __name__ == '__main__':
    rostopic, rosmsg_type = pickle.loads(sys.argv[1])
    listen(rostopic, rosmsg_type)

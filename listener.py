#! /usr/bin/env python

"""A ros node that will subscribe to a single message and print it to stdout.
"""
import cPickle as pickle
import sys

import rospy


def listen(topic, msg_type):
    """A function to isolate some of the subscriber logic.
    """
    rospy.init_node('mock_listen_'+topic.split('/')[-1], anonymous=True,
            disable_signals=True)
    def listener_cb(message):
        """A default callback for our subscriber.
        """
        message.serialize(sys.stdout)
        rospy.signal_shutdown('message sent')
    rospy.Subscriber(topic, msg_type, listener_cb)
    rospy.spin()



if __name__ == '__main__':
    ROSTOPIC, ROSMSG_TYPE = pickle.loads(sys.argv[1])
    listen(ROSTOPIC, ROSMSG_TYPE)

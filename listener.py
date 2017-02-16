#! /usr/bin/env python

"""A ros node that will subscribe to a single message and print it to stdout.

It takes in pickled configuration string that should be the ROSTOPIC and
ROSMSG_TYPE pickled on the other end. This also means that this node must be
run from a context where it can import any of the buzzmobile ros messages.

It prints the recieved data to stdout serialized for use by the main thread.
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

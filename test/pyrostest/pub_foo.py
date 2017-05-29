#!/usr/bin/env python

import time

import rospy
from std_msgs.msg import Int32

pub = rospy.Publisher('/pub_val', Int32, queue_size=10)


def pub_foo():
    rospy.init_node('pub_foo', anonymous=True)
    for _ in range(100):
        pub.publish(rospy.get_param('/foo'))
        time.sleep(1)


if __name__ == '__main__':
    pub_foo()

#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

pub = rospy.Publisher('/pub_val', Int32, queue_size=10)


def pub_foo():
    rospy.init_node('pub_foo', anonymous=True)
    while True:
        pub.publish(rospy.get_param('/foo'))


if __name__ == '__main__':
    pub_foo()

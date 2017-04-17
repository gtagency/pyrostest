#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

pub = rospy.Publisher('add_one', Int32, queue_size=10)

def add_one_cb(val):
    pub.publish(Int32(val.data + 1))

def add_one():
    rospy.init_node('adder', anonymous=True)
    rospy.Subscriber('subber', Int32, add_one_cb)
    rospy.spin()

if __name__ == '__main__':
    add_one()

#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String


def callback(msg):
    rospy.loginfo("Duydum: %s" % msg.data)


def main():
    rospy.init_node("listener")

    sub = rospy.Subscriber("/chatter_topic", String, callback)

    rospy.spin()

if __name__ == '__main__':
    main()

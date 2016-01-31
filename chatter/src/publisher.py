#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node("talker")
    pub = rospy.Publisher("/chatter_topic", String, queue_size=1)
    rate = rospy.Rate(10) # 10 Hertz ile çalışıyor

    while not rospy.is_shutdown():
        message = "Naber Dünya? %s" % (rospy.get_time())
        rospy.loginfo("Mesaj hazırlandı: %s" % message)
        pub.publish(message)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

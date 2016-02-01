#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
import tf
import cv2
import cv_bridge
import math


def sign(x):
    return math.copysign(1, x)


class Point(object):
    def __init__(self, pose, radius, spread, point_type):
        self.pose = pose
        self.radius = radius
        self.spread = spread
        self.point_type = point_type

    def get_vector(self, position):
        dist = ((self.pose[0] - position[0]) ** 2 + (self.pose[1] - position[1]) ** 2) ** 0.5
        angle = math.atan2(self.pose[1] - position[1], self.pose[0] - position[0])

        dx = dy = 0.0

        if self.point_type == 'ATTRACTIVE':
            if dist < self.radius:
                dx = dy = 0.0
            elif self.radius <= dist < self.radius + self.spread:
                dx = (dist - self.radius) * math.cos(angle)
                dy = (dist - self.radius) * math.sin(angle)
            elif dist >= self.radius + self.spread:
                dx = self.spread * math.cos(angle)
                dy = self.spread * math.sin(angle)
        elif self.point_type == 'REPULSIVE':
            if dist < self.radius:
                dx = -sign(math.cos(angle))
                dy = -sign(math.sin(angle))
            elif self.radius <= dist < self.radius + self.spread:
                dx = (self.spread + self.radius - dist) * math.cos(angle)
                dy = (self.spread + self.radius - dist) * math.sin(angle)
            elif dist >= self.radius + self.spread:
                dx = dy = 0.0

        return dx, dy


class Robot(object):
    def __init__(self, name):
        self.name = name

        # Publisher
        self.cmd_vel = rospy.Publisher("/%s/cmd_vel" % self.name,
                                       Twist, queue_size=1)

        # Subscriber
        self.odom = rospy.Subscriber("/%s/odom" % self.name,
                                     Odometry, self.odom_callback,
                                     queue_size=1)

        self.laser = rospy.Subscriber("/%s/front_laser/scan" % self.name,
                                      LaserScan, self.laser_callback,
                                      queue_size=1)

        self.camera = rospy.Subscriber("/%s/front_camera/image_raw" % self.name,
                                       Image, self.camera_callback,
                                       queue_size=1)

        self.pose_data = None
        self.laser_data = None
        self.camera_data = None

        self.cv_bridge = cv_bridge.CvBridge()

        self.rate = rospy.Rate(10)
        self.rate.sleep()

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        quat = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        euler = tf.transformations.euler_from_quaternion(quat)

        self.pose_data = [
            position.x,
            position.y,
            euler[2]
        ]


    def laser_callback(self, msg):
        self.laser_data = msg.ranges

    def camera_callback(self, msg):
        try:
            self.camera_data = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except cv_bridge.CvBridgeError:
            return

        gray = cv2.cvtColor(self.camera_data, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        canny = cv2.Canny(blur, 30, 150)

        cv2.imshow("Robot Camera", canny)
        cv2.waitKey(1)


    def set_speed(self, linear, angular):
        movecmd = Twist()
        movecmd.linear.x = linear
        movecmd.angular.z = angular
        self.cmd_vel.publish(movecmd)


    def stop(self):
        self.set_speed(0.0, 0.0)


    def pf_test(self):
        attractive = Point((5, 5), 0.05, 1.0, 'ATTRACTIVE')
        repulsive = Point((2.5, 3.5), 0.10, 0.9, 'REPULSIVE')

        completed = False

        while not completed:
            rx, ry, rt = self.pose_data

            attr_force = attractive.get_vector((rx, ry))
            rep_force = repulsive.get_vector((rx, ry))

            vx = attr_force[0] + rep_force[0] * 2.0
            vy = attr_force[1] + rep_force[1] * 2.0

            if attr_force[0] == 0.0 and attr_force[1] == 0.0:
                self.stop()
                completed = True
            else:
                linear = 0.55
                angle = math.atan2(vy, vx)

                if angle == 0.0:
                    angular = 0.0
                else:
                    angular = angle - rt

                self.set_speed(linear, angular)

            self.rate.sleep()


    def goto_point(self, tx, ty):
        completed = False

        while not completed:
            rx, ry, rt = self.pose_data

            dist = ((tx - rx) ** 2 + (ty - ry) ** 2) ** 0.5
            angle = math.atan2(ty - ry, tx - rx)

            radius = dist * 0.5 / math.sin(rt - angle) if rt - angle != 0.0 else float('Inf')

            linear = angular = 0.0

            if dist > 0.05:
                linear = abs(radius) if abs(radius) < 1.0 else 1.0
                angular = -linear / radius
                angular *= 2.5
                self.set_speed(linear, angular)
            else:
                self.stop()
                completed = True

            self.rate.sleep()


def main():
    rospy.init_node("robot_controller")

    r = Robot("p3dx")

    r.pf_test()



if __name__ == '__main__':
    main()

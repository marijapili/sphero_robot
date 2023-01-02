#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math

import rospy
from geometry_msgs.msg import Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler


class MarkerServer(object):
    def __init__(self):
        self.num_robots = rospy.get_param('/num_of_robots', 1)
        self.robot_name = rospy.get_param('/robot_name', 'sphero')

        self.markers_pub = rospy.Publisher('/markers', MarkerArray, queue_size=1)

        self.init_kalman()

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            all_markers = MarkerArray()
            all_markers.markers.extend(self.kalman_markers)

            self.markers_pub.publish(all_markers)

            r.sleep()

    def init_kalman(self):
        self.kalman_markers = []
        for i in range(self.num_robots):
            m = Marker()
            m.header.frame_id = f'{self.robot_name}_{i}/base_link'
            m.header.stamp = rospy.get_rostime()
            m.ns = 'kalman'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose = Pose()
            m.lifetime = rospy.Duration(0)
            m.frame_locked = True
            m.color = ColorRGBA(1, 0.55, 0, 1)  # orange
            self.kalman_markers.append(m)

        self.kalman_subs = [
            rospy.Subscriber(f'{self.robot_name}_{i}/odom_est', Odometry, self.kalman_callback,
                             callback_args=i, queue_size=1)
            for i in range(self.num_robots)
        ]

    def kalman_callback(self, data: Odometry, args):
        angle = Quaternion(
            *quaternion_from_euler(0, 0, math.atan2(data.twist.twist.linear.y, data.twist.twist.linear.x))
        )
        scale = Vector3(math.sqrt(data.twist.twist.linear.y ** 2 + data.twist.twist.linear.x ** 2), 0.02, 0.02)

        self.kalman_markers[args].header.stamp = rospy.get_rostime()
        self.kalman_markers[args].pose.orientation = angle
        self.kalman_markers[args].scale = scale


if __name__ == "__main__":
    rospy.init_node("marker_server")

    try:
        node = MarkerServer()
    except rospy.ROSInterruptException:
        pass

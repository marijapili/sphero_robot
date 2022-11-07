#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import message_filters as mf
import numpy as np

from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

from sort import Sort


def np_to_points(np_array):
    x1, y1, x2, y2, id = np_array

    points = [Point(x1, y1, 0),
              Point(x2, y1, 0),
              Point(x2, y2, 0),
              Point(x1, y2, 0)]

    return points, id


def create_markers(points, id):
    box = Marker()
    box.header.frame_id = 'map'
    box.header.stamp = rospy.get_rostime()
    box.ns = 'tracked_bb'
    box.id = id
    box.type = Marker.LINE_STRIP
    box.action = Marker.ADD
    box.pose = Pose()
    box.scale.x = 0.01
    box.color = ColorRGBA(0, 1, 0, 1)
    box.lifetime = rospy.Duration(0.2)
    box.points = points
    box.frame_locked = True

    text = Marker()
    text.header.frame_id = 'map'
    text.header.stamp = rospy.get_rostime()
    text.ns = 'tracked_id'
    text.id = id
    text.type = Marker.TEXT_VIEW_FACING
    text.action = Marker.ADD
    text.pose = Pose(points[0], Quaternion())
    text.scale.z = 0.25
    text.text = str(int(id))
    text.color = ColorRGBA(1, 0, 0, 1)
    text.lifetime = rospy.Duration(0.2)
    text.frame_locked = True

    return box, text


class SORTwrapper(object):
    def __init__(self):
        # Load parameters and create a tracker object.
        max_age = rospy.get_param('~max_age', 3)  # Default: 1
        min_hits = rospy.get_param('~min_hits', 3)  # Default: 3
        iou_threshold = rospy.get_param('~iou_threshold', 0.15)  # Default: 0.3
        self.bb_size = rospy.get_param('~bb_size', 0.2)
        self.tracker = Sort(max_age=max_age, min_hits=min_hits, iou_threshold=iou_threshold)

        # Publishers for the tracked data.
        self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=10)

        # Case-specific parameters and data subscribers.
        self.num_agents = 1
        self.first = True
        # robot_name = rospy.get_param('/robot_name')

        # subs = [mf.Subscriber(robot_name + '_{}/odom'.format(i), Odometry) for i in range(self.num_agents)]
        # self.ts = mf.ApproximateTimeSynchronizer(subs, 10, 0.11)  # TODO: set this to be parametric as well
        # self.ts.registerCallback(self.update_cb)
        self.pub1 = rospy.Publisher('sphero_0/odom', Odometry, queue_size=1)
        self.pub2 = rospy.Publisher('sphero_1/odom', Odometry, queue_size=1)
        self.pub3 = rospy.Publisher('sphero_2/odom', Odometry, queue_size=1)

    def update_cb(self, data):
        # Convert input data to numpy array.
        detections = np.ndarray((len(data), 5))
        for i, obj in enumerate(data):
            x1 = obj[0] - self.bb_size
            x2 = obj[0] + self.bb_size
            y1 = obj[1] - self.bb_size
            y2 = obj[1] + self.bb_size
            score = 1
            detections[i] = np.array([x1, y1, x2, y2, score])

        # Update the tracker
        tracked = self.tracker.update(detections)

        for obj in tracked:
            points, id = np_to_points(obj)
            temp_msg = Odometry()
            temp_msg.pose.pose.position.x = points[0].x + self.bb_size
            temp_msg.pose.pose.position.y = points[0].y + self.bb_size
            if len(data) == 3:
                if id == 1:
                    temp_msg.header.frame_id = 'sphero_0'
                    self.pub1.publish(temp_msg)
                elif id == 2:
                    temp_msg.header.frame_id = 'sphero_1'
                    self.pub2.publish(temp_msg)
                else:
                    temp_msg.header.frame_id = 'sphero_2'
                    self.pub3.publish(temp_msg)

        # Publish tracked objects.
        marker_array = MarkerArray()
        for obj in tracked:
            points, id = np_to_points(obj)
            bbox, name = create_markers(points, id)
            marker_array.markers.append(bbox)
            marker_array.markers.append(name)
        self.markers_pub.publish(marker_array)

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
    x1, y1, x2, y2, ID = np_array

    points = [Point(x1, y1, 0),
              Point(x2, y1, 0),
              Point(x2, y2, 0),
              Point(x1, y2, 0)]

    return points, ID


def create_markers(points, ID):
    box = Marker()
    box.header.frame_id = 'map'
    box.header.stamp = rospy.get_rostime()
    box.ns = 'tracked_bb'
    box.id = ID
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
    text.id = ID
    text.type = Marker.TEXT_VIEW_FACING
    text.action = Marker.ADD
    text.pose = Pose(points[0], Quaternion())
    text.scale.z = 0.25
    text.text = str(int(ID))
    text.color = ColorRGBA(1, 0, 0, 1)
    text.lifetime = rospy.Duration(0.2)
    text.frame_locked = True

    return box, text


class SORTwrapper(object):
    def __init__(self, max_age=3, min_hits=3, iou_threshold=0.15, bb_size=0.2, num_robots=1):
        # Load parameters and create a tracker object.
        max_age = rospy.get_param('~max_age', max_age)
        min_hits = rospy.get_param('~min_hits', min_hits)
        iou_threshold = rospy.get_param('~iou_threshold', iou_threshold)
        self.bb_size = rospy.get_param('~bb_size', bb_size)
        self.tracker = Sort(max_age=max_age, min_hits=min_hits, iou_threshold=iou_threshold)

        # Publishers for the tracked data.
        self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=10)

        # Case-specific parameters and data subscribers.
        self.num_robots = rospy.get_param('/num_of_robots', num_robots)
        self.robot_name = rospy.get_param('/robot_name', 'sphero')

        # subs = [mf.Subscriber(robot_name + '_{}/odom'.format(i), Odometry) for i in range(self.num_agents)]
        # self.ts = mf.ApproximateTimeSynchronizer(subs, 10, 0.11)  # TODO: set this to be parametric as well
        # self.ts.registerCallback(self.update_cb)
        self.pubs = [rospy.Publisher(f'/{self.robot_name}_{i}/position', Point, queue_size=1)
                     for i in range(self.num_robots)]

    # TODO: make it general so it can be called directly or as a callback
    def update_cb(self, data):
        """
        Update tracker with new measurements.
        
        params:
            data (list[tuple[float, float]]): Centers of measurements.
        """
        # Convert input data to numpy array.
        detections = np.ndarray((len(data), 5))
        for i, obj in enumerate(data):
            x1 = obj[0] - self.bb_size
            x2 = obj[0] + self.bb_size
            y1 = obj[1] - self.bb_size
            y2 = obj[1] + self.bb_size
            score = 1
            detections[i] = np.array([x1, y1, x2, y2, score])

        # Update the tracker.
        tracked = self.tracker.update(detections)
        
        marker_array = MarkerArray()
        for obj in tracked:
            points, id = np_to_points(obj)
            
            # Publish positions of tracked objects.
            temp_msg = Point()
            temp_msg.x = points[0].x + self.bb_size
            temp_msg.y = points[0].y + self.bb_size
            # TODO: publish even if no tracking
            if len(tracked) == self.num_robots:
                self.pubs[int(id - 1)].publish(temp_msg)

            # Publish visualization of tracked objects.
            bbox, name = create_markers(points, id)
            marker_array.markers.append(bbox)
            marker_array.markers.append(name)
        
        self.markers_pub.publish(marker_array)

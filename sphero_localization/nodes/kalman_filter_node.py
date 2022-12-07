#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tf
import rospy
import math
from geometry_msgs.msg import PoseArray, Point
from nav_msgs.msg import Odometry

from sphero_localization.kalman_filter import KalmanFilter
from sphero_localization.srv import *

def pose_dist(pose1, pose2):
    """Return Euclidean distance between two ROS poses."""
    x1 = pose1.position.x
    y1 = pose1.position.y
    x2 = pose2.position.x
    y2 = pose2.position.y

    return math.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))


class KalmanFilterNode(object):
    """
    ROS node implementation of Kalman filter.

    This node subscribes to a list of all existing Sphero's positions
    broadcast from OptiTrack system, associates one of them to the Sphero in
    the same namespace and uses Kalman filter to output steady position and
    velocity data for other nodes.
    """

    def __init__(self):
        """Initialize agent instance, create subscribers and publishers."""
        # Initialize class variables
        self.missing_counter = 0   # Counts iterations with missing marker information
        self.pub_frequency = rospy.get_param('/ctrl_loop_freq')
        self.sub_frequency = rospy.get_param('/data_stream_freq')
        self.debug_enabled = rospy.get_param('/debug_kalman')
        self.ns = rospy.get_namespace().strip('/')

        self.filter = None
        self.initial_position = None

        # Create a publisher for commands.
        pub = rospy.Publisher('odom_est', Odometry, queue_size=self.pub_frequency)
        if self.debug_enabled:
            # Debug publisher runs at the same frequency as incoming data.
            self.debug_pub = rospy.Publisher('debug_est', Odometry, queue_size=self.sub_frequency)

        # Create subscribers.
        rospy.Subscriber('position', Point, self.position_callback, queue_size=self.sub_frequency)

        # Get the initial positions of the robots.
        self.get_initial_position()
        rospy.loginfo(rospy.get_namespace() + ' Initial position:\n%s\n', self.initial_position)

        # Initialize Kalman filter and estimation
        self.filter = KalmanFilter(1.0 / self.sub_frequency, self.initial_position)
        self.X_est = Odometry()
        self.X_est.pose.pose.position = self.initial_position

        # Create tf broadcaster
        br = tf.TransformBroadcaster()

        # Main while loop.
        rate = rospy.Rate(self.pub_frequency)
        while not rospy.is_shutdown():
            pub.publish(self.X_est)
            pos = self.X_est.pose.pose.position
            br.sendTransform((pos.x, pos.y, pos.z),
                             (0, 0, 0, 1),
                             rospy.Time.now(),
                             self.ns + '/base_link',
                             'map')
            rospy.logdebug(' x = % 7.5f', self.X_est.pose.pose.position.x)
            rospy.logdebug(' y = % 7.5f', self.X_est.pose.pose.position.y)
            rospy.logdebug('vx = % 7.5f', self.X_est.twist.twist.linear.x)
            rospy.logdebug('vy = % 7.5f\n', self.X_est.twist.twist.linear.y)
            rate.sleep()

    def get_initial_position(self):
        while self.initial_position is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def position_callback(self, data):
        """Process received positions data and return Kalman estimation."""
        if self.initial_position is None:
            self.initial_position = data

        if self.filter is None:
            return

        # Get measurement
        X_measured = data
        time = rospy.Time.now()

        # If measurement data is not available, use only prediction step.
        # Otherwise use prediction and update step.
        # TODO: Switch to fixed rate loop instead of data callback.
        #       With webcam tracking and SORT, if this happens, it means we lost
        #       the track and have to start again. However, for general case, 
        #       we should still have the option of doing only predection.
        #       We can track if a new message arrived in the current step
        #       interval inside the data callback, but this method should run
        #       with fixed rate. 
        if X_measured is None:
            self.X_est = self.filter.predict()
        else:
            self.X_est = self.filter.predict_update(X_measured)

        self.X_est.header.stamp = time
        self.X_est.header.frame_id = self.ns + '/base_link'

        if self.debug_enabled:
            self.debug_pub.publish(self.X_est)


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('Kalman')

    # Go to class functions that do all the heavy lifting
    # Do error checking
    try:
        kf = KalmanFilterNode()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tf2_ros
import rospy
import threading
from geometry_msgs.msg import PoseArray, Point, TransformStamped, Quaternion
from nav_msgs.msg import Odometry

from sphero_localization.kalman_filter import KalmanFilter


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
        self.data_stream_freq = rospy.get_param('/data_stream_freq')
        self.debug_enabled = rospy.get_param('/debug_kalman')
        self.ns = rospy.get_namespace().strip('/')

        self.filter = None
        self.initial_position = None

        # Create a publisher for commands.
        pub = rospy.Publisher('odom_est', Odometry, queue_size=self.data_stream_freq)
        if self.debug_enabled:
            # Debug publisher runs at the same frequency as incoming data.
            self.debug_pub = rospy.Publisher('debug_est', Odometry, queue_size=self.data_stream_freq)

        self.lock = threading.Lock()

        # Create subscribers.
        rospy.Subscriber('position', Point, self.position_callback, queue_size=self.data_stream_freq)

        # Get the initial positions of the robots.
        self.get_initial_position()
        rospy.loginfo(rospy.get_namespace() + ' Initial position:\n%s\n', self.initial_position)

        # Initialize Kalman filter and estimation
        self.filter = KalmanFilter(1.0 / self.data_stream_freq, self.initial_position)
        self.X_est = Odometry()
        self.X_est.pose.pose.position = self.initial_position
        self.last_measurement = None

        # Create tf broadcaster
        br = tf2_ros.TransformBroadcaster()

        # Main while loop.
        rate = rospy.Rate(self.data_stream_freq)
        
        while not rospy.is_shutdown():
            # Compute the predict step.
            self.X_est = self.filter.predict()
            
            # Compute the update step.
            with self.lock:
                if self.last_measurement is not None:
                    self.X_est = self.filter.update(self.last_measurement)
                    self.last_measurement = None
                    
            # Publish the current estimate. 
            self.X_est.header.stamp = rospy.Time.now()
            self.X_est.header.frame_id = self.ns + '/base_link'     
            pub.publish(self.X_est)
            
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = self.ns + '/base_link'
            t.transform.translation.x = self.X_est.pose.pose.position.x
            t.transform.translation.y = self.X_est.pose.pose.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation = Quaternion(0, 0, 0, 1)
            br.sendTransform(t)
            
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
        with self.lock:
            self.last_measurement = data


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('Kalman')

    # Go to class functions that do all the heavy lifting
    # Do error checking
    try:
        kf = KalmanFilterNode()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry

from pid import PID


class SpheroPositionPID(object):
    def __init__(self):
        # Initialize class variables.
        self.cmd_vel = Twist()
        self.position = Point()
        self.reference = Point()

        # Define subscribers and publishers.
        pub = rospy.Publisher('pid_vel', Twist, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('reference', Point, self.ref_cb, queue_size=1)

        # Set PID parameters.
        Kp = 0.25 * 255
        Ki = 0.0005 * 255
        Kd = 0
        Td = 0.15
        lim_lo = -40
        lim_hi = 40
        tol = 0.1

        # Initialize PID controllers.
        self.pid_x = PID(Kp, Ki, Kd, Td, lim_lo, lim_hi, tol)
        self.pid_y = PID(Kp, Ki, Kd, Td, lim_lo, lim_hi, tol)

        rate = rospy.Rate(1/Td)
        while not rospy.is_shutdown():
            self.cmd_vel.linear.x = self.pid_x.compute(self.reference.x, self.position.x)
            self.cmd_vel.linear.y = self.pid_y.compute(self.reference.y, self.position.y)
            pub.publish(self.cmd_vel)
            rate.sleep()

    def odom_cb(self, msg):
        """
        Save the current Sphero's position to class variable.

        Args:
            msg (Odometry): Current Sphero odometry.
        """
        self.position = msg.pose.pose.position

    def ref_cb(self, msg):
        """
        Save the commanded reference position to class variable.

        Args:
            msg (Odometry): Current Sphero odometry.
        """
        self.reference = msg


if __name__ == "__main__":
    rospy.init_node("sphero_pid_control")

    try:
        node = SpheroPositionPID()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import copy
import math
import sys

from spherov2 import scanner as SpheroScanner
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.types import Color as SpheroColor
from spherov2.commands.power import BatteryStates as SpheroBatteryStates

import rospy
import tf2_ros
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Pose2D, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, ColorRGBA, Float32, Header
from tf.transformations import quaternion_from_euler

from sphero_constants import *

# TODO: cathc concurrent.futures._base.TimeoutError
# FIXME: high cpu usage

class SpheroNode(object):
    def __init__(self):
        # Basic parameters
        self.ns = rospy.get_namespace().strip('/')
        self.sphero_name = rospy.get_param("~name", "SK-7519")
        
        # Connection setttings
        self.connection_attempts = rospy.get_param("~connection_attempts", 3)
        self.connection_timeout = rospy.get_param("~connection_timeout", 5.0)
        self.connect_color = SpheroColor(0, 100, 0)
        
        # Other parameters
        self.cmd_vel_timeout = rospy.Duration(nsecs=rospy.get_param('~cmd_vel_timeout', 2.0) * 1e9)
        self.batt_pub_interval = rospy.Duration(nsecs=5.0 * 1e9)
        
        # Robot APIs
        self.robot_llc = None
        self.robot_api = None
        
        # Robot state
        self.is_connected = False
        self.is_enabled = False
        self.cmd_speed = 0
        self.cmd_heading = 0
        self.last_cmd_heading = 0
        self.last_cmd_vel_time = rospy.Time.now()
        self.last_batt_pub_time = rospy.Time.now()
        self.odom = Odometry(header=Header(frame_id="odom"), child_frame_id=f'{self.ns}/base_footprint')
        self.odom_tare = Odometry()
        
        # Subscribers
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel, queue_size=1)
        self.heading_sub = rospy.Subscriber('set_heading', Float32, self.set_heading, queue_size=1)
        self.color_sub = rospy.Subscriber('set_color', ColorRGBA, self.set_color, queue_size=1)
        self.manual_calibration_sub = rospy.Subscriber('manual_calibration', Bool,
                                                       self.manual_calibration, queue_size=1)
        self.stabilization_sub = rospy.Subscriber('set_stabilization', Bool,
                                                  self.set_stabilization, queue_size=1)
        # Publishers
        self.batt_pub = rospy.Publisher('battery', DiagnosticStatus, queue_size=1)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
        
        # TF broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.odom_tf = TransformStamped(header=Header(frame_id="odom"), child_frame_id=f'{self.ns}/base_footprint')
        
        # Timers
        self.batt_timer = rospy.Timer(self.batt_pub_interval, self.get_battery_state)
        
    ## Start, stop, and spin methods.
    def start(self):
        attempts_left = self.connection_attempts
        delay = float(self.ns.split('_')[-1]) * 2
        rospy.sleep(delay)
        
        rospy.loginfo("Trying to connect.")
        while attempts_left > 0:
            try:
                self.robot_llc = SpheroScanner.find_toy(toy_name=self.sphero_name, timeout=1.0)
                break
            except SpheroScanner.ToyNotFoundError:
                attempts_left -= 1
                if attempts_left > 0:
                    rospy.logwarn(f"Can't find Sphero {self.sphero_name}. Trying again {attempts_left} times.")
                else:
                    rospy.logerr(f"There is no Sphero {self.sphero_name}. Giving up.")
                    sys.exit(1)
        
        rospy.loginfo(f"Connected to Sphero '{self.ns}' with BT name {self.sphero_name}.")
        
        self.robot_api = SpheroEduAPI(self.robot_llc)  # FIXME: sometimes dies with bleak.exc.BleakDBusError: [org.bluez.Error.InProgress] In Progress
        self.robot_api.__enter__()  # FIXME Then this dies with concurrent.futures._base.TimeoutError
        
        rospy.sleep(1.0)
        
        self.robot_api.set_main_led(self.connect_color)
        self.is_enabled = True
        self.is_connected = True
        rospy.loginfo(f"Sphero '{self.ns}' with BT name {self.sphero_name} is ready.")
        
    def spin(self):
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown() and self.is_connected:
            now = rospy.Time.now()
            # If there is no command for velocity longer than 5 seconds, stop the Sphero.
            if (now - self.last_cmd_vel_time) > self.cmd_vel_timeout and self.is_enabled:
                if self.cmd_heading != 0 or self.cmd_speed != 0:
                    self.cmd_heading = 0
                    self.cmd_speed = 0
                    rospy.logwarn("No cmd_vel received in %.2f seconds. Emergency stop!", self.cmd_vel_timeout.to_sec())
                    self.roll(self.cmd_speed, self.cmd_heading)
            
            # Publish sensor readings.
            self.get_imu()
            self.get_odometry()
            # if (now - self.last_batt_pub_time) > self.batt_pub_interval:
            #     self.last_batt_pub_time = now
            #     self.get_battery_state()
            
            r.sleep()
            
    def stop(self):
        if self.is_connected:
            self.is_connected = False
            rospy.loginfo('Stopping.')
            rospy.sleep(2.0)
            rospy.loginfo('Disconnecting from the robot.')
            self.batt_timer.shutdown()
            self.robot_api.__exit__(None, None, None)
        else:
            rospy.loginfo('Robot already disconnected.')
    
    ## ROS callback -> Sphero command
    def cmd_vel(self, msg: Twist):
        """Send command for setting the velocity."""
        self.last_cmd_vel_time = rospy.Time.now()
        if self.is_enabled and self.is_connected:
            if msg.linear.x == 0 and msg.linear.y == 0:
                self.cmd_heading = self.last_cmd_heading
            else:
                self.cmd_heading = SpheroNode.normalize_angle_positive(
                    math.atan2(msg.linear.x, msg.linear.y)) * 180 / math.pi
                self.last_cmd_heading = self.cmd_heading
            self.cmd_speed = math.sqrt(math.pow(msg.linear.x, 2) + math.pow(msg.linear.y, 2))
    
            self.roll(self.cmd_speed, self.cmd_heading)
            
    def set_heading(self, msg: Float32):
        """Send command for setting the velocity."""
        heading_deg = int(self.normalize_angle_positive(math.radians(msg.data)) * 180.0 / math.pi)
        self.robot_api.set_heading(heading_deg)
        
    def set_color(self, msg: ColorRGBA):
        """Send command for setting the color."""
        self.robot_api.set_main_led(SpheroColor(int(msg.r * 255), int(msg.g * 255), int(msg.b * 255)))
        
    def manual_calibration(self, msg: Bool):
        """Enable or disable manual heading calibration."""
        if msg.data:
            self.is_enabled = False
            self.last_cmd_heading = 0
            self.robot_api.set_speed(0)
            self.robot_api.set_stabilization(False)  # Disable stabilization.
            self.robot_api.set_back_led(255)  # Turn on tail light.
            rospy.logdebug(" Manual calibration mode ON")
        elif not msg.data:
            self.robot_api.reset_aim()
            self.robot_api.set_back_led(0)  # turn off tail light
            self.is_enabled = True
            rospy.logdebug(" Manual calibration mode OFF")

            self.odom_tare.pose.pose.position.x = self.odom.pose.pose.position.x
            self.odom_tare.pose.pose.position.y = self.odom.pose.pose.position.y

    def set_stabilization(self, msg: Bool):
        """Send command for stabilization control."""
        if not msg.data:
            self.robot_api.set_stabilization(True)  # enable stabilization
        else:
            self.robot_api.set_stabilization(False)  # disable stabilization
            
    ## Sphero reading -> ROS publisher
    def get_battery_state(self, timer_event):
        """Get battery state from Sphero and publish."""
        if not self.is_enabled:
            return
        
        battery = self.robot_api.get_battery()
        
        batt_msg = DiagnosticStatus()
        batt_msg.name = "Battery Status"
        batt_msg.level = BATT_STATE_MAP[battery.state]
        batt_msg.message = f"Battery {battery.state.name}"
        batt_msg.values = [KeyValue(k, str(v)) for k, v in battery.__dict__.items() if k != 'state']
        
        if batt_msg.level == DiagnosticStatus.WARN:
            rospy.logwarn_throttle(30, batt_msg.message)
        elif batt_msg.level == DiagnosticStatus.ERROR:
            rospy.logerr_throttle(30, batt_msg.message)
        
        self.batt_pub.publish(batt_msg)
    
    def get_imu(self):
        """Get IMU measurements from Sphero and publish."""
        imu_msg = Imu(header=rospy.Header(frame_id=f"{self.ns}/base_link"))
        
        acc = self.robot_api.get_acceleration()
        gyr = self.robot_api.get_gyroscope()
        ori = self.robot_api.get_orientation()
        
        imu_msg.header.stamp = rospy.Time.now()
        try:
            imu_msg.orientation = Quaternion(
                *quaternion_from_euler(math.radians(ori['roll']),
                                       math.radians(ori['pitch']),
                                       math.radians(ori['yaw']))
            )
        except TypeError:
            rospy.logwarn_throttle(1, "Orientation measurements not available!")
            imu_msg.orientation_covariance[0] = -1
        
        try:    
            imu_msg.linear_acceleration = Vector3(acc['x'] * 9.80665, acc['y'] * 9.80665, acc['z'] * 9.80665)
        except TypeError:
            rospy.logwarn_throttle(1, "Acceleration measurements not available!")
            imu_msg.linear_acceleration_covariance[0] = -1
        
        try:
            imu_msg.angular_velocity = Vector3(math.radians(gyr['x']), math.radians(gyr['y']), math.radians(gyr['z']))
        except TypeError:
            rospy.logwarn_throttle(1, "Angular velocity measurements not available!")
            imu_msg.angular_velocity_covariance[0] = -1
        
        self.imu_pub.publish(imu_msg)
        
    def get_odometry(self):
        """Get odometry data from Sphero and publish."""
        odom_msg = Odometry()
        odom_msg.header = self.odom.header
        odom_msg.child_frame_id = self.odom.child_frame_id
        
        loc = self.robot_api.get_location()
        ori = self.robot_api.get_orientation()
        vel = self.robot_api.get_velocity()
        
        try:
            odom_msg.pose.pose.position = Point(loc['x'] / 100, loc['y'] / 100, 0)
            odom_msg.pose.pose.orientation = Quaternion(
                *quaternion_from_euler(math.radians(ori['pitch']),
                                       math.radians(ori['roll']),
                                       math.radians(ori['yaw']))
            )
            odom_msg.twist.twist.linear = Vector3(vel['x'] / 100, vel['y'] / 100, 0)
        except TypeError:
            rospy.logwarn_throttle(1, "Odometry measurements not available!")
            odom_msg = copy.deepcopy(self.odom)
            
        odom_msg.header.stamp = rospy.Time.now()
        
        self.odom.pose.pose.position.x = odom_msg.pose.pose.position.x
        self.odom.pose.pose.position.y = odom_msg.pose.pose.position.y
        
        odom_msg.pose.pose.position.x -= self.odom_tare.pose.pose.position.x
        odom_msg.pose.pose.position.y -= self.odom_tare.pose.pose.position.y
        self.odom_pub.publish(odom_msg)
        
        self.odom_tf.header.stamp = odom_msg.header.stamp
        self.odom_tf.transform.translation = odom_msg.pose.pose.position
        self.odom_tf.transform.rotation = odom_msg.pose.pose.orientation
        self.broadcaster.sendTransform(self.odom_tf)
            
    ## Helper functions        
    def roll(self, speed: float, heading: float):
        """
        Simultaneously set Sphero's speed and heading.
        
        params:
            speed (float): Speed in range 0-255
            heading (float): Heading in range 0-360
        """
        # FIXME: This sometimes times out
        self.robot_api.set_heading(int(heading))
        self.robot_api.set_speed(int(speed))
        
    @staticmethod
    def normalize_angle_positive(angle):
        """Return positive angle in radians, in range [0, 2pi]."""
        return math.fmod(math.fmod(angle, 2.0 * math.pi) + 2.0 * math.pi, 2.0 * math.pi)
    

if __name__ == "__main__":
    rospy.init_node("driver")
    
    try:
        node = SpheroNode()
        rospy.on_shutdown(node.stop)
        node.start()
        node.spin()
    except rospy.ROSInterruptException as e:
        pass
    #     print(f'ROS exception\n{e}')
    #     node.stop()
    # finally:
    #     print('Shutting down normally.')
    #     node.stop()
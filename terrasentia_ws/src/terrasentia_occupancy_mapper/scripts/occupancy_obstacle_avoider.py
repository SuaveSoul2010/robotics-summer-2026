#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist

class RoombaNavigator:
    def __init__(self):
        rospy.init_node("roomba_navigator")

        # Navigation / Motion Parameters
        self.linear_speed = 0.2  # Forward velocity (m/s)
        self.angular_speed = 0.5  # Rotation velocity (rad/s)

        self.safety_dist_x = 0.6  # How far ahead to look for obstacles (meters)
        self.safety_dist_y = 0.3  # Width of the clearance corridor (meters)

        # State tracking: "FORWARD" or "TURNING"
        self.state = "FORWARD"

        # Subscriber Data Storage
        self.grid_msg = None
        self.grid_received = False

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.pose_received = False

        # Subscribers & Publishers Listens to the map published by your mapper node
        self.map_sub = rospy.Subscriber(
            "/occupancy_grid", OccupancyGrid, self.map_callback, queue_size=1
        )

        # Listens to odometry for current robot orientation
        self.odom_sub = rospy.Subscriber(
            "/robot/odometry/local/raw", Odometry, self.odom_callback, queue_size=1
        )

        # Publishes velocity commands to the robot base
        self.cmd_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)

        # Main control loop running at 10 Hz
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

        rospy.loginfo("Roomba Navigator Node initialized.")

    # Quaternion -> Yaw
    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # Callbacks
    def map_callback(self, msg):
        self.grid_msg = msg
        self.grid_received = True

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.robot_theta = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.pose_received = True

    # Obstacle Check
    def is_path_blocked(self):
        """
        Inspects cells in the OccupancyGrid that fall within a bounding box
        directly ahead of the robot frame.
        """
        if not self.grid_received or self.grid_msg is None:
            return False

        info = self.grid_msg.info
        res = info.resolution
        w = info.width
        h = info.height
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        # Convert flat list data into 2D array (height x width)
        grid_data = np.array(self.grid_msg.data, dtype=np.int8).reshape((h, w))

        # Check cells in a bounding grid area forward of the robot Map world points inside the local corridor around the robot
        cos_t = math.cos(self.robot_theta)
        sin_t = math.sin(self.robot_theta)

        # Step through a bounding box in front of the robot (local frame)
        dx_steps = np.arange(0.1, self.safety_dist_x, res)
        dy_steps = np.arange(-self.safety_dist_y / 2.0, self.safety_dist_y / 2.0, res)

        for dx in dx_steps:
            for dy in dy_steps:
                # Transform local offset into global world coordinates
                wx = self.robot_x + (dx * cos_t - dy * sin_t)
                wy = self.robot_y + (dx * sin_t + dy * cos_t)

                # World -> Grid Indices
                gx = int((wx - origin_x) / res)
                gy = int((wy - origin_y) / res)

                # Check grid boundaries
                if 0 <= gx < w and 0 <= gy < h:
                    # Value > 50 indicates occupied cell
                    if grid_data[gy, gx] > 50:
                        return True

        return False

    # Control Loop (10 Hz)
    def control_loop(self, event):

        # System Checks
        if not self.pose_received:
            rospy.logwarn_throttle(
                2.0, "Navigator: Waiting for odometry on /robot/odometry/local/raw..."
            )
            self.stop_robot()
            return

        if not self.grid_received:
            rospy.logwarn_throttle(
                2.0, "Navigator: Waiting for map on /occupancy_grid..."
            )
            self.stop_robot()
            return

        # Behavior Logic
        cmd = Twist()
        blocked = self.is_path_blocked()

        if self.state == "FORWARD":
            if blocked:
                rospy.loginfo("Obstacle detected! Switching state to TURNING.")
                self.state = "TURNING"
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed
            else:
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0

        elif self.state == "TURNING":
            if not blocked:
                rospy.loginfo("Path clear! Switching state to FORWARD.")
                self.state = "FORWARD"
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0
            else:
                # Keep turning until space opens up
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

# Main
def main():
    try:
        RoombaNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

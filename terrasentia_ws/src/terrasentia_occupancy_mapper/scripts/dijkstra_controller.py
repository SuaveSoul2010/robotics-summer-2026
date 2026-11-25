#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
import tf.transformations

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path


class PathFollowerController:
    def __init__(self):
        rospy.init_node("path_follower_controller")

        # Control Parameters
        self.lookahead_distance = 0.4  # Meters ahead on path to target
        self.goal_tolerance = 0.2  # Distance (m) to consider final goal reached
        self.max_linear_speed = 0.25  # m/s
        self.max_angular_speed = 0.8  # rad/s
        self.kp_angular = 1.5  # Proportional gain for heading correction

        # Robot Pose State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.pose_received = False

        # Path State
        self.path_waypoints = []
        self.goal_reached = True

        # Subscribers & Publishers
        self.odom_sub = rospy.Subscriber(
            "/robot/odometry/local/raw", Odometry, self.odom_callback, queue_size=1
        )

        self.path_sub = rospy.Subscriber(
            "/calculated_path", Path, self.path_callback, queue_size=1
        )

        self.cmd_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)

        # Control Loop Timer (20 Hz)
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)

        rospy.loginfo("Path Follower Controller initialized and ready.")

    # Callbacks
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract yaw angle from orientation quaternion
        orientation_q = msg.pose.pose.orientation
        quaternion = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        _, _, self.robot_yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.pose_received = True

    def path_callback(self, msg):
        if not msg.poses:
            rospy.logwarn("Received empty path.")
            return

        self.path_waypoints = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self.goal_reached = False
        rospy.loginfo(
            "New path received with %d waypoints. Starting execution.",
            len(self.path_waypoints),
        )

    # Control Loop Logic
    def control_loop(self, event):
        if not self.pose_received or self.goal_reached or not self.path_waypoints:
            return

        # Check distance to final goal waypoint
        final_x, final_y = self.path_waypoints[-1]
        dist_to_final = math.hypot(final_x - self.robot_x, final_y - self.robot_y)

        if dist_to_final < self.goal_tolerance:
            rospy.loginfo("Goal reached successfully! Stopping robot.")
            self.stop_robot()
            self.goal_reached = True
            return

        # Select target lookahead waypoint along path
        target_x, target_y = self.get_lookahead_point()

        # Calculate relative angle to lookahead target
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        target_heading = math.atan2(dy, dx)

        # Normalize heading error to [-pi, pi]
        heading_error = target_heading - self.robot_yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # Compute Twist commands
        cmd = Twist()

        # Proportional angular velocity steering control
        cmd.angular.z = np.clip(
            self.kp_angular * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        # Scale forward velocity down during steep turns to prevent overshooting
        turn_reduction = max(0.0, 1.0 - (abs(heading_error) / (math.pi / 2.0)))
        cmd.linear.x = self.max_linear_speed * turn_reduction

        self.cmd_pub.publish(cmd)

    def get_lookahead_point(self):
        """Finds the path point located closest to the lookahead distance."""
        for wx, wy in self.path_waypoints:
            dist = math.hypot(wx - self.robot_x, wy - self.robot_y)
            if dist >= self.lookahead_distance:
                return wx, wy
        # Fallback to final waypoint if no point satisfies lookahead criteria
        return self.path_waypoints[-1]

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


if __name__ == "__main__":
    try:
        PathFollowerController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

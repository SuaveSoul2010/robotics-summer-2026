#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge

# Occupancy grid parameters
LOG_ODDS_FREE = -0.4
LOG_ODDS_OCCUPIED = 0.85
LOG_ODDS_MIN = -5.0
LOG_ODDS_MAX = 5.0
LOG_ODDS_THRESHOLD = 0.0


class TerraSentiaOccupancyMapper:
    def __init__(self):
        rospy.init_node("terrasentia_occupancy_mapper")
        self.bridge = CvBridge()

        # Subscribers
        self.depth_sub = rospy.Subscriber(
            "/camera/depth/image_rect_raw", Image, self.depth_callback, queue_size=1
        )

        self.odom_sub = rospy.Subscriber(
            "/robot/odometry/local/raw", Odometry, self.odom_callback, queue_size=1
        )

        self.camera_info_sub = rospy.Subscriber(
            "/camera/depth/camera_info",
            CameraInfo,
            self.camera_info_callback,
            queue_size=1,
        )

        # Publishers Occupancy grid for RViz / Navigation (latched so subscribers receive it immediately)
        self.map_pub = rospy.Publisher(
            "/occupancy_grid", OccupancyGrid, queue_size=1, latch=True
        )

        # Goal pose
        self.goal_pub = rospy.Publisher("/goal_pose", PoseStamped, queue_size=1)

        # Robot position + heading for RViz
        self.pose_marker_pub = rospy.Publisher(
            "/robot_pose_marker", Marker, queue_size=1
        )

        # Robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.pose_received = False

        # Camera parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_received = False

        # Goal
        self.goal_x = None
        self.goal_y = None

        # Map parameters
        self.map_size_x = 20.0
        self.map_size_y = 20.0
        self.resolution = 0.05
        self.max_mapping_range = 8.0
        self.grid_width = int(self.map_size_x / self.resolution)
        self.grid_height = int(self.map_size_y / self.resolution)

        # Log-odds map
        self.log_odds = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)

        # Publish an initial empty map immediately upon node start
        self.publish_map()

        # Mapping rate limiter
        self.last_map_time = rospy.Time.now()

        self.map_period = 0.1
        # 10 Hz

        # Startup messages
        rospy.loginfo("TerraSentia occupancy mapper started")
        rospy.loginfo("Grid size: %d x %d", self.grid_width, self.grid_height)

    # Quaternion -> yaw
    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # Odometry callback
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_theta = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.pose_received = True

        # Publish robot marker continuously from odometry. This means the RViz arrow updates even when the depth mapper is rate-limited.
        self.publish_robot_pose_marker()

    # Camera info callback
    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]

        self.cx = msg.K[2]
        self.cy = msg.K[5]

        self.camera_info_received = True

        rospy.loginfo_once(
            "Camera intrinsics received: "
            "fx=%.2f fy=%.2f cx=%.2f cy=%.2f" % (self.fx, self.fy, self.cx, self.cy)
        )

    # World -> grid
    def world_to_grid(self, x, y):
        grid_x = int((x + self.map_size_x / 2.0) / self.resolution)
        grid_y = int((y + self.map_size_y / 2.0) / self.resolution)

        return grid_x, grid_y

    # Check grid bounds
    def in_bounds(self, grid_x, grid_y):
        return 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height

    # Bresenham line
    def bresenham_line(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x = x0
        y = y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        if dx > dy:
            err = dx / 2.0

            while x != x1:
                points.append((x, y))
                err -= dy

                if err < 0:
                    y += sy
                    err += dx

                x += sx

        else:
            err = dy / 2.0

            while y != y1:
                points.append((x, y))
                err -= dx

                if err < 0:
                    x += sx
                    err += dy

                y += sy

        points.append((x1, y1))

        return points

    # Update free cell
    def update_free_cell(self, grid_x, grid_y):
        if not self.in_bounds(grid_x, grid_y):
            return

        self.log_odds[grid_y, grid_x] = max(
            LOG_ODDS_MIN, self.log_odds[grid_y, grid_x] + LOG_ODDS_FREE
        )

    # Update occupied cell
    def update_occupied_cell(self, grid_x, grid_y):
        if not self.in_bounds(grid_x, grid_y):
            return

        self.log_odds[grid_y, grid_x] = min(
            LOG_ODDS_MAX, self.log_odds[grid_y, grid_x] + LOG_ODDS_OCCUPIED
        )

    # Depth callback
    def depth_callback(self, msg):
        # Need odometry first
        if not self.pose_received:
            rospy.logwarn_throttle(
                2.0, "OccupancyMapper: Waiting for odometry message..."
            )
            return

        # Need camera intrinsics
        if not self.camera_info_received:
            rospy.logwarn_throttle(
                2.0, "OccupancyMapper: Waiting for camera info message..."
            )
            return

        # Limit mapping rate
        now = rospy.Time.now()

        if (now - self.last_map_time).to_sec() < self.map_period:

            return

        self.last_map_time = now

        # Convert ROS image to NumPy array
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        except Exception as e:
            rospy.logerr("CV Bridge error: %s", e)

            return

        # Image dimensions
        h, w = depth.shape

        # Approximate camera position. Camera is assumed to be 0.12 m forward of the robot center.
        camera_x = self.robot_x + 0.12 * math.cos(self.robot_theta)
        camera_y = self.robot_y + 0.12 * math.sin(self.robot_theta)

        # Camera position in grid
        camera_grid_x, camera_grid_y = self.world_to_grid(camera_x, camera_y)

        if not self.in_bounds(camera_grid_x, camera_grid_y):
            return

        # Use a horizontal band around the middle of the depth image. This approximates a 2D scan.
        band_top = int(h * 0.40)
        band_bottom = int(h * 0.60)
        band = depth[band_top:band_bottom, :]

        # Process every 8th horizontal pixel.
        pixel_step = 8

        for u in range(0, w, pixel_step):
            # Get vertical column
            column = band[:, u]

            # Keep finite values
            valid = column[np.isfinite(column)]

            # Remove zero depth values
            valid = valid[valid > 0]

            if len(valid) == 0:
                continue

            # Median depth
            depth_value = np.median(valid)

            # RealSense depth is normally millimeters.
            distance = float(depth_value) / 1000.0

            # Ignore invalid / extremely close values.
            if distance <= 0.05:
                continue

            # Ignore points outside mapping range.
            if distance > self.max_mapping_range:
                continue

            # Horizontal camera angle
            angle = math.atan2(float(u - self.cx), self.fx)

            # Camera-local coordinates local_x = forward local_y = left/right
            local_x = distance * math.cos(angle)
            local_y = distance * math.sin(angle)

            # Camera-local -> world
            world_x = (
                camera_x
                + local_x * math.cos(self.robot_theta)
                - local_y * math.sin(self.robot_theta)
            )

            world_y = (
                camera_y
                + local_x * math.sin(self.robot_theta)
                + local_y * math.cos(self.robot_theta)
            )

            # World -> grid
            endpoint_grid_x, endpoint_grid_y = self.world_to_grid(world_x, world_y)

            if not self.in_bounds(endpoint_grid_x, endpoint_grid_y):
                continue

            # Trace ray
            ray = self.bresenham_line(
                camera_grid_x, camera_grid_y, endpoint_grid_x, endpoint_grid_y
            )

            # Cells before obstacle = free
            for free_x, free_y in ray[:-1]:
                self.update_free_cell(free_x, free_y)

            # Endpoint = occupied
            self.update_occupied_cell(endpoint_grid_x, endpoint_grid_y)

        # Publish updated map
        self.publish_map()

    # Publish robot pose / heading marker
    def publish_robot_pose_marker(self):
        if not self.pose_received:
            return

        marker = Marker()

        # Frame
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "odom"

        # Marker identification
        marker.ns = "robot_pose"
        marker.id = 0

        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Robot position
        marker.pose.position.x = self.robot_x
        marker.pose.position.y = self.robot_y
        marker.pose.position.z = 0.20

        # Robot orientation yaw -> quaternion
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0

        marker.pose.orientation.z = math.sin(self.robot_theta / 2.0)
        marker.pose.orientation.w = math.cos(self.robot_theta / 2.0)

        # Arrow dimensions Arrow points along +X. The quaternion rotates +X into robot heading.
        marker.scale.x = 0.8
        marker.scale.y = 0.15
        marker.scale.z = 0.15

        # Red
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Marker persists until replaced
        marker.lifetime = rospy.Duration(0)

        self.pose_marker_pub.publish(marker)

    # Publish OccupancyGrid
    def publish_map(self):
        msg = OccupancyGrid()

        # Header
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"

        # Map metadata
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height

        # Map origin Center the 20m x 20m map around (0,0).
        msg.info.origin.position.x = -self.map_size_x / 2.0
        msg.info.origin.position.y = -self.map_size_y / 2.0
        msg.info.origin.position.z = 0.0

        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Unknown = -1 Free = 0 Occupied = 100
        grid_out = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)

        # Occupied
        grid_out[self.log_odds > LOG_ODDS_THRESHOLD] = 100

        # Free
        grid_out[self.log_odds < 0.0] = 0

        # OccupancyGrid uses row-major order
        msg.data = grid_out.flatten().tolist()

        # Publish
        self.map_pub.publish(msg)


# Main
def main():
    try:

        TerraSentiaOccupancyMapper()
        rospy.spin()

    except rospy.ROSInterruptException:

        pass

if __name__ == "__main__":
    main()
